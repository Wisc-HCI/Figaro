package com.example.temideploy;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;

import androidx.core.app.ActivityCompat;

import com.robotemi.sdk.Robot;
import com.robotemi.sdk.TtsRequest;
import com.robotemi.sdk.listeners.OnDetectionStateChangedListener;
import com.robotemi.sdk.listeners.OnGoToLocationStatusChangedListener;
import com.robotemi.sdk.listeners.OnRobotReadyListener;
import com.robotemi.sdk.permission.Permission;

import org.jetbrains.annotations.NotNull;
import org.ros.android.AppCompatRosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.w3c.dom.Document;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Stack;

public class MainActivity extends AppCompatRosActivity implements //AppCompatActivity implements
        OnDetectionStateChangedListener,
        OnRobotReadyListener,
        OnGoToLocationStatusChangedListener,
        TiltObserver,
        PointObserver,
        WaitObserver,
        View.OnClickListener
{
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static final int VOICE_RECOGNITION_REQUEST_CODE = 1234;
    private ListView mList;
    private static String[] PERMISSIONS_STORAGE = {Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE};
    public EditText etSpeak, etSaveLocation, etGoTo;
    public DeployClient deploy;
    // declare the robot
    private Robot robot;
    private TransitionSystem ts;
    private Stack<TransitionSystem> ts_stack = new Stack<TransitionSystem>();
    private HashMap<String,HashMap<ArrayList<String>,TransitionSystem>> interrupts;
    private EnvironmentalState es;
    private  TextView tv1;
    private ImageView iv1;
    private TextView statusView;
    private Button startButton;
    private Button endButton;
    private SpeechDatabase sd;
    private boolean triggerFreeze;
    private boolean debug;
    String history;

    // detect human timer
    private DetectionTimer dtimer;
    private boolean goLock;

    // speech buttons
    ArrayList<Button> speechBtns;
    ImageView btnView;

    // camera + orientation
    CameraController cam;
    private OrientationManager orman;
    private TiltManager tman;
    private PointManager pman;
    private WaitManager wman;
    private static final int MY_CAMERA_REQUEST_CODE = 100;
    private static final int MY_SETTINGS_REQUEST_CODE = 101;

    // keep track of locations to the best of our ability
    private String currLocation;

    // movement pausing
    private boolean movementPaused;
    private String pausedMovementDestination;

    // declare communication framework
    private PubSubComm pubsub;

    private boolean startable;
    private boolean resettingInteraction;

    // debug mode variables
    private TextView currExecutionStateView;
    private TextView currSensingView;
    private TextView currTimerView;
    private ArrayList<String> executionPath;

    // firebase
    private FirebaseServer fb_client;

    // ros
    private RosNode rosnode;

    // TODO: convert this database to XML so that it isn't hard-coded
    private HashMap<String,HashMap<String,Double>> directionMapping;

    /**
     * Called upon initialization
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_HIDDEN);
        verifyStoragePermissions(this);
        verifyCameraPermissions(this);

        this.dtimer = null;
        this.startable = false;
        this.goLock = false;
        this.resettingInteraction = false;

        this.tv1 = (TextView)findViewById(R.id.textView2);
        this.tv1.setText("");
        this.iv1 = (ImageView)findViewById(R.id.imageView2);
        this.startButton = (Button)findViewById(R.id.button3);
        this.endButton = (Button)findViewById(R.id.button2);
        this.statusView = (TextView)findViewById(R.id.textView3);

        this.speechBtns = new ArrayList<Button>();
        this.speechBtns.add((Button)findViewById(R.id.button5));
        this.speechBtns.add((Button)findViewById(R.id.button6));
        this.speechBtns.add((Button)findViewById(R.id.button7));
        this.speechBtns.add((Button)findViewById(R.id.button8));
        this.speechBtns.add((Button)findViewById(R.id.button9));
        this.speechBtns.add((Button)findViewById(R.id.button10));
        this.speechBtns.add((Button)findViewById(R.id.button11));
        this.speechBtns.add((Button)findViewById(R.id.button12));
        this.speechBtns.add((Button)findViewById(R.id.button13));
        this.speechBtns.add((Button)findViewById(R.id.button14));
        this.speechBtns.add((Button)findViewById(R.id.button15));
        this.speechBtns.add((Button)findViewById(R.id.button16));
        for (int i = 0; i < 12; i++) {
            this.speechBtns.get(i).setVisibility(View.GONE);
        }
        this.btnView = (ImageView)findViewById(R.id.imageView4);
        this.btnView.setVisibility(View.GONE);

        this.startButton.setOnClickListener(this);
        this.startButton.setEnabled(true);
        this.endButton.setOnClickListener(this);
        this.endButton.setEnabled(false);
        this.statusView.setText("");

        // print the working directory
        String currentDirectory = System.getProperty("user.dir");
        Log.d("CREATION","The current working directory is " + currentDirectory);

        // load the main transition system
        Context context = this;
        sd = new SpeechDatabase();
        sd.readSpeechXml(context, "speech.xml");
        Log.d("LOADING",sd.toString());
        this.loadInteraction();

        // load the interrupts
        this.clearInterrupts();
        AssetManager am = context.getAssets();
        try {
            String[] interruptFiles = am.list("interrupts");
            Log.d("STATUS", "loading interrupts folder");
            for (String name : interruptFiles) {
                // extract the event from the name
                String event = name.substring(12,name.length()-4);
                int idx = event.lastIndexOf("_");
                event = event.substring(0,idx);
                TransitionSystem interrupt = new TransitionSystem(context,this);
                String loadName = "interrupts/" + name;
                interrupt.loadTransitionSystemFile(loadName);

                // extract the value of the event
                ArrayList<String> vals = interrupt.getInterruptValue();

                if (this.interrupts.containsKey(event)) {
                    this.interrupts.get(event).put(vals,interrupt);
                }
                else {
                    HashMap<ArrayList<String>, TransitionSystem> valsMap = new HashMap<ArrayList<String>, TransitionSystem>();
                    valsMap.put(vals, interrupt);
                    this.interrupts.put(event, valsMap);
                }
                Log.d("STATUS","added an interrupt for event " + event);


                for (State state : interrupt.states) {
                    state.interrupts = new HashMap<String,HashMap<ArrayList<String>,TransitionSystem>>();
                }


            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        // run through every state in the main transition system, updating the interrupt data
        for (State state : this.ts.states) {
            state.interrupts = this.interrupts;
        }

        // initialize the environmental state
        ArrayList<String> envCats = new ArrayList<String>();
        envCats.add("position");
        envCats.add("close_to_human");
        this.es = new EnvironmentalState(envCats);
        this.es.putState("position",new ArrayList<String>());
        ArrayList<String> temp = new ArrayList<String>();
        temp.add("false");
        this.es.putState("close_to_human",temp);
        Log.d("ENVIRONMENT",this.es.toString());

        // initialize the robot
        robot = Robot.getInstance(); // get an instance of the robot in order to begin using its features.

        this.triggerFreeze = false;

        // keeping track of locations
        this.currLocation = "default";

        // movement pausing
        this.movementPaused = false;
        this.pausedMovementDestination = null;

        // initialize debugging vars
        this.debug = true;
        this.history = "";

        // initialize view components
        this.currExecutionStateView = (TextView)findViewById(R.id.textViewState);
        this.currExecutionStateView.setText("");
        this.currSensingView = (TextView)findViewById(R.id.textViewSense);
        this.currSensingView.setText("");
        this.currTimerView = (TextView)findViewById(R.id.textViewTimer);
        this.currTimerView.setText("");
        this.executionPath = new ArrayList<String>();

        // update the debug view
        this.updateSenseView(this.es.toString());

        // initialize the firebase server
        //this.fb_client = new FirebaseServer();
        //this.fb_client.initializeTopic();

        // initialize the camera
        cam = new CameraController(getApplicationContext());

        // initialize the custom behavior
        this.tman = new TiltManager(robot);
        this.pman = new PointManager(robot);
        this.wman = new WaitManager(robot);

        // direction mappings
        // TODO: convert this database to XML so that it isn't hard-coded
        directionMapping = new HashMap<String,HashMap<String,Double>>();
        directionMapping.put("exhibit1",new HashMap<String,Double>());
        directionMapping.put("exhibit2",new HashMap<String,Double>());
        directionMapping.put("exhibit3",new HashMap<String,Double>());
        directionMapping.put("exhibit4", new HashMap<String,Double>());

        directionMapping.get("exhibit1").put("exhibit1", 0.0);
        directionMapping.get("exhibit1").put("exhibit2",300.0);
        directionMapping.get("exhibit1").put("exhibit3",345.0);
        directionMapping.get("exhibit1").put("exhibit4",70.0);

        directionMapping.get("exhibit2").put("exhibit1",70.0);
        directionMapping.get("exhibit2").put("exhibit2",0.0);
        directionMapping.get("exhibit2").put("exhibit3",300.0);
        directionMapping.get("exhibit2").put("exhibit4",345.0);

        directionMapping.get("exhibit3").put("exhibit1",345.0);
        directionMapping.get("exhibit3").put("exhibit2",70.0);
        directionMapping.get("exhibit3").put("exhibit3",0.0);
        directionMapping.get("exhibit3").put("exhibit4",300.0);

        directionMapping.get("exhibit4").put("exhibit1",300.0);
        directionMapping.get("exhibit4").put("exhibit2",345.0);
        directionMapping.get("exhibit4").put("exhibit3",70.0);
        directionMapping.get("exhibit4").put("exhibit4",0.0);
        /*
        directionMapping.put("americangothic",new HashMap<String,Double>());
        directionMapping.put("circuitcity",new HashMap<String,Double>());
        directionMapping.put("mrmittens",new HashMap<String,Double>());
        directionMapping.put("default", new HashMap<String,Double>());
        directionMapping.get("americangothic").put("circuitcity", 90.0);
        directionMapping.get("americangothic").put("mrmittens",135.0);
        directionMapping.get("americangothic").put("americangothic",270.0);
        directionMapping.get("circuitcity").put("mrmittens",180.0);
        directionMapping.get("circuitcity").put("americangothic",270.0);
        directionMapping.get("circuitcity").put("circuitcity",45.0);
        directionMapping.get("mrmittens").put("circuitcity",20.0);
        directionMapping.get("mrmittens").put("americangothic",335.0);
        directionMapping.get("mrmittens").put("mrmittens",90.0);
        directionMapping.get("default").put("mrmittens",20.0);
        directionMapping.get("default").put("americangothic",340.0);
        directionMapping.get("default").put("circuitcity",20.0);
        */

        // activate robot detection
        //this.robot.setDetectionModeOn(true);
    }

    public void toggleDebug(boolean val) {
        this.debug = val;
        this.updateSenseView(this.es.toString());
        this.updateStateView(this.ts.getCurrState().toStringAbridged());
    }

    public void listenForInteraction() {
        this.deploy.load();
    }

    public void loadInteraction() {
        Context context = this;
        this.ts = new TransitionSystem(context,this);
        this.ts.loadTransitionSystemFile("interaction__self.xml");
        this.ts.reset();
    }

    public void loadInteraction(Document doc) {
        Context context = this;
        this.ts = new TransitionSystem(context,this);
        this.ts.loadTransitionSystem(doc);
        this.ts.reset();
    }

    public void loadSpeechExamples(Document doc) {
        sd.readSpeechDocument(doc);
    }

    public void loadInterrupt(Document doc) {
        Context context = this;
        TransitionSystem interrupt = new TransitionSystem(context,this);
        interrupt.loadTransitionSystem(doc);

        String event = interrupt.getInterruptName();
        event = event.substring(12,event.length());
        int idx = event.lastIndexOf("_");
        event = event.substring(0,idx);

        // extract the value of the event
        ArrayList<String> vals = interrupt.getInterruptValue();

        if (this.interrupts.containsKey(event)) {
            this.interrupts.get(event).put(vals,interrupt);
        }
        else {
            HashMap<ArrayList<String>, TransitionSystem> valsMap = new HashMap<ArrayList<String>, TransitionSystem>();
            valsMap.put(vals, interrupt);
            this.interrupts.put(event, valsMap);
        }
        Log.d("STATUS","added an interrupt for event " + event);

        // INTERRUPT DEPTH OF 1
        for (State state : interrupt.states) {
            state.interrupts = new HashMap<String,HashMap<ArrayList<String>,TransitionSystem>>();
        }

        // run through every state in the main transition system, updating the interrupt data
        for (State state : this.ts.states) {
            state.interrupts = this.interrupts;
        }
    }

    public void clearInterrupts() {
        this.interrupts = new HashMap<String,HashMap<ArrayList<String>,TransitionSystem>>();
    }

    /**
     * Checks if the app has permission to write to device storage
     * If the app does not has permission then the user will be prompted to grant permissions
     */
    public static void verifyStoragePermissions(Activity activity) {
        // Check if we have write permission
        int permission = ActivityCompat.checkSelfPermission(activity, Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (permission != PackageManager.PERMISSION_GRANTED) {
            // We don't have permission so prompt the user
            ActivityCompat.requestPermissions(activity, PERMISSIONS_STORAGE, REQUEST_EXTERNAL_STORAGE);
        }
    }

    /**
     * Checks if the app has permission to use the camera in the background
     * If the app does not has permission then the user will be prompted to grant permissions
     */
    public static void verifyCameraPermissions(Activity activity) {
        // Check if we have write permission
        int permission = ActivityCompat.checkSelfPermission(activity, Manifest.permission.CAMERA);
        if (permission != PackageManager.PERMISSION_GRANTED) {
            // We don't have permission so prompt the user
            ActivityCompat.requestPermissions(activity,new String[]{Manifest.permission.CAMERA},
                    MY_CAMERA_REQUEST_CODE );
        }
    }

    /**
     * Checks if the app has permission to change system settings
     * If the app does not have permission, then the user will be prompted to grant permission
     */
    public static void verifySettingsPermissions() {
        // Check if we have write permission
        int permission = Robot.getInstance().checkSelfPermission(Permission.SETTINGS);
        if (permission != Permission.GRANTED) {
            Log.d("PERMISSIONS","not yet granted!");
            // We don't have permission so prompt the user
            List<Permission> p = new ArrayList<Permission>();
            p.add(Permission.SETTINGS);
            Robot.getInstance().requestPermissions(p,MY_SETTINGS_REQUEST_CODE);
        }
        else {
            Log.d("PERMISSIONS","already granted.");
        }
    }

    /**
     * Setting up all the event listeners
     */
    @Override
    protected void onStart() {
        super.onStart();
        Log.d("STATUS","starting");
        robot.addOnDetectionStateChangedListener(this);
        robot.addOnGoToLocationStatusChangedListener(this);
        this.tman.registerObserver(this);
        this.pman.registerObserver(this);
        this.wman.registerObserver(this);
        robot.toggleWakeup(true);

        /*
        LocalBroadcastManager.getInstance(this).registerReceiver((mMessageReceiver),
                new IntentFilter("MyData")
        );
         */
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        rosnode = new RosNode(this);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(rosnode, nodeConfiguration);
        verifySettingsPermissions();
        boolean result = this.robot.isSelectedKioskApp();
        Log.d("Is kiosk app?", String.valueOf(result));
        if (!result)
            this.robot.requestToBeKioskApp();
        this.robot.setDetectionModeOn(false);
        this.orman = new OrientationManager(robot,cam,this);
    }



    /**
     * Removing the event listeners upon leaving the app.
     */
    @Override
    protected void onStop() {
        super.onStop();
        Log.d("STATUS","stopping");
        //LocalBroadcastManager.getInstance(this).unregisterReceiver(mMessageReceiver);
    }

    /**
     * Places this application in the top bar for a quick access shortcut.
     */
    @Override
    public void onRobotReady(boolean isReady) {
        if (isReady) {
            try {
                final ActivityInfo activityInfo = getPackageManager().getActivityInfo(getComponentName(), PackageManager.GET_META_DATA);
                robot.onStart(activityInfo);
            } catch (PackageManager.NameNotFoundException e) {
                throw new RuntimeException(e);
            }
        }
    }

    /**
     * Set interaction to the initial state and begin
     */
    public void resetInteraction() {
        // reset the button
        this.startButton.setEnabled(false);
        this.endButton.setEnabled(true);

        // remove any dtimers
        if (this.dtimer != null) {
            this.dtimer.isActive = false;
        }

        // update the environmental state
        ArrayList<String> temp = new ArrayList<String>();
        temp.add("false");
        this.es.putState("close_to_human", temp);
        setStatusText("");

        // get a fresh slate
        this.ts.reset();
        this.history = "";

        // make temi go to "default" location
        this.resettingInteraction = true;
        temiGo("default");
    }

    public void finishReset() {
        // update the state view
        this.updateStateView(this.ts.getCurrState().toStringAbridged());

        // artificially execute first behavior
        this.ts.executeCurr();
        ArrayList<String> behVal = new ArrayList<String>();
        behVal.add("ON");
        this.ts.tellStateBehaviorDone("sys",behVal);

        // trigger the first state change
        HashMap<String,ArrayList<String>> initEvent = new HashMap<String, ArrayList<String>>();
        ArrayList<String> selfEvent = new ArrayList<String>();
        selfEvent.add("self");
        initEvent.put("",selfEvent);

        // can start now
        this.startable = true;
        this.triggerStateChange(initEvent);
    }

    /**
     * Halt current interaction.
     */
    public void endInteraction() {
        // reset the buttons
        this.startButton.setEnabled(true);
        this.endButton.setEnabled(false);

        // terminate all threads
        for (State st : this.ts.states) {
            st.halt();
        }
        for (Map.Entry<String, HashMap<ArrayList<String>, TransitionSystem>> entry : this.interrupts.entrySet()) {
            HashMap<ArrayList<String>, TransitionSystem> temp = entry.getValue();
            for (Map.Entry<ArrayList<String>,TransitionSystem> secondEntry : temp.entrySet()) {
                TransitionSystem interrupt = secondEntry.getValue();
                for (State st : interrupt.states)
                    st.halt();
            }
        }

        temiGo("default");
        Log.d("HISTORY",this.history);

        this.startable = false;
        //System.exit(0);
    }

    /**
     * Exit program.
     */
    public void shutdown() {
        System.exit(0);
    }

    /**
     * Event occurred.
     * Attempt to trigger a state change caused by the event.
     */
    public void triggerStateChange(HashMap<String,ArrayList<String>> event) {

        // prevent trigger if there is a trigger freeze
        if (this.triggerFreeze || !this.startable)// || this.goLock)
            return;

        Log.d("EXECUTION","Attempting to trigger a state change");
        HashMap<String,ArrayList<String>> envState = this.es.getEnvironmentalState();
        Log.d("EXECUTION EVENT", String.valueOf(event));
        for (Map.Entry<String, ArrayList<String>> entry : envState.entrySet()) {
            Log.d("EXECUTION ENV","\n    " + entry.getKey() + " = " + entry.getValue());
        }
        State newState = this.ts.triggerChange(event,envState);
        if (newState != null) {
            Log.d("DISPLAY","removing speech options");
            //this.removeSpeechOptions();
            prepareStateExecution(newState);
        }
    }

    /**
     * Switch execution to a different transition system (e.g. an interrupt)
     * Place new TS on the stack.
     */
    public void switchTs(TransitionSystem other_ts) {
        this.ts_stack.push(this.ts);
        this.ts = other_ts;
    }

    /**
     * Pop the execution stack and resume the previous transition system
     */
    public void resumeTs() {
        Log.d("EXECUTION","resuming old TS");
        this.ts = this.ts_stack.pop();
    }

    /**
     * Preprocess state before executing.
     */
    public void prepareStateExecution(State newState) {
        // before entering a new state, movement needs to be removed
        HashMap<String,ArrayList<String>> currEnvState = this.es.getEnvironmentalState();
        ArrayList<String> positionState = currEnvState.get("position");
        if (positionState.contains("movement")) {
            positionState.remove("movement");
        }

        // now execute
        this.executeState(newState);
    }

    /**
     * Execute Temi actions based on the current state.
     */
    public void executeState(State state) {

        this.pausedMovementDestination = null;

        HashMap<String,ArrayList<String>> behaviors = state.behaviors;
        Log.d("EXECUTION", "attempting to execute the following behaviors in state " + state.id);
        Log.d("EXECUTION",state.toString());
        Log.d("EXECUTION", String.valueOf(behaviors));
        for (Map.Entry<String, ArrayList<String>> entry : behaviors.entrySet()) {
            String cat = entry.getKey();

            // IF WAIT, ALWAYS GO
            // if cat has wait, ALWAYS resume it
            if (cat.equals("wait")) {
                // verbalize point
                //TtsRequest ttsRequest = TtsRequest.create("I am waiting", true);
                //robot.speak(ttsRequest);

                // immediately return from the behavior
                this.setStatusText("waiting");
                this.waitFinished();
            }

            // IF NOT WAIT AND BEHAVIORS DONE, SKIP!
            // don't resume already completed behaviors
            else if (state.behaviorsDone.contains(cat) && !(cat.equals("movement") && entry.getValue().size()>0))
                continue;

            Log.d("EXECUTION", "executing behavior " + cat);
            ArrayList<String> params = entry.getValue();

            // now look at the rest of the behaviors
            if (cat.equals("speech")) {
                String val = params.get(0);
                this.temiSay(val);
            }
            else if (cat.equals("tilt")) {
                String direction = params.get(0);
                //long val = Long.parseLong(params.get(1));
                //this.robot.setDetectionModeOn(false);
                //this.tman.tilt(direction,val);
                this.tman.tilt(direction,6000);

                // verbalize tilt
                //TtsRequest ttsRequest = TtsRequest.create("I am tilting " + 6000, true);
                //robot.speak(ttsRequest);

                this.setStatusText("tilting " + direction);
            }
            else if (cat.equals("point")) {
                //pause any movement
                this.temiStop();
                this.movementPaused = true;

                String region = params.get(0);
                HashMap<String,ArrayList<String>> currEnvState = this.es.getEnvironmentalState();
                ArrayList<String> positionState = currEnvState.get("position");
                String position;
                if (positionState.size() == 0)
                    position = "default";
                else
                    position = positionState.get(0);

                // UNCOMMENT1
                Double direction = this.directionMapping.get(position).get(region);

                // get current orientation
                //UNCOMMENT 2
                //double currOrientation = this.orman.getCurrOrientation();
                //Log.d("ORIENTATION", String.valueOf(currOrientation));


                //this.robot.setDetectionModeOn(false);

                // verbalize point
                //TtsRequest ttsRequest = TtsRequest.create("I am pointing to " + region, true);
                //robot.speak(ttsRequest);
                //this.pman.point(currOrientation, direction);

                // point differently based on the location of Temi
                this.pman.point(0.0,direction);

                this.setStatusText("pointing " + region);
            }
            else if (cat.equals("actions")) {
                String val = params.get(0);

            }
            else if (cat.equals("movement")) {

                // it is POSSIBLE (and likely) that movement does not have an argument
                if (params.size() > 0) {
                    String val = params.get(0);
                    this.pausedMovementDestination = val;

                    if (!this.movementPaused) {
                        Log.d("STATUS", "initiating movement to " + val);
                        this.temiGo(val);
                    }

                }
            }
        }
    }

    /**
     * * * * * * *
     * EVENT LISTENERS
     * * * * * * *
     */

    /**
     * Temi heard speech.
     */
    public void temiListen(String speechCategory) {
        Log.d("TEMI HEARD",speechCategory + " was heard by Temi");
        setStatusText("I heard you!");
        // trigger an event
        Log.d("EVENT","h1_speech is " + speechCategory + "\n" + this.es.toString());
        HashMap<String,ArrayList<String>> event = new HashMap<String,ArrayList<String>>(); // package up the event HashMap
        ArrayList<String> eventVal = new ArrayList<String>();
        eventVal.add(speechCategory);
        event.put("h1_speech",eventVal);
        triggerStateChange(event);
    }

    /**
     * Temi detected a change in state for human proximity
     */
    @Override
    public void onDetectionStateChanged(int i) {
        Log.d("HUMAN", String.valueOf(i));
        //this.robot.setDetectionModeOn(false);
        //this.tman.maintainTiltAngle();
        if (i == 2) {

            // trigger an event
            if (this.dtimer != null) {
                if (!this.dtimer.isActive) {
                    this.dtimer = new DetectionTimer(this);
                    this.dtimer.start();
                }
            } else {
                this.dtimer = new DetectionTimer(this);
                this.dtimer.start();
            }

        } else {

            setStatusText("I don't see you!");
            // remove any dtimers
            if (this.dtimer != null) {
                this.dtimer.isActive = false;
            }

            // update the environmental state
            ArrayList<String> temp = new ArrayList<String>();
            temp.add("false");
            this.es.putState("close_to_human", temp);
            setStatusText("");
        }

        // update the debug view
        this.updateSenseView(this.es.toString());
    }

    public void triggerH1NearRobot() {
        // update the environmental state
        ArrayList<String> temp = new ArrayList<String>();
        temp.add("true");
        this.es.putState("close_to_human", temp);

        // trigger an event
        Log.d("EVENT", "h1_near_rob is True\n" + this.es.toString());
        HashMap<String, ArrayList<String>> event = new HashMap<String, ArrayList<String>>(); // package up the event HashMap
        ArrayList<String> eventVal = new ArrayList<String>();
        eventVal.add("true");
        event.put("h1_near_rob", eventVal);
        triggerStateChange(event);
    }

    /**
     *ENVIRONMENTAL STATE LISTENERS
     *
     * Currently the only one listens for changes in Temi's location status
     */
    @Override
    public void onGoToLocationStatusChanged(@NotNull String s, @NotNull String s1, int i, @NotNull String s2) {
        Log.d("LOCATION CHANGE",s1 + " " + s + " " + s2);

        if (s1.equals("going") || s1.equals("start") || s1.equals("calculating")) {
            Log.d("GLOCK","setting the go lock");
            this.goLock = true;
        }

        // ignore aborted! (which happens in an interrupt)
        if (s1.equals("complete")) {

            // update the curr location
            this.currLocation = s;

            Log.d("GLOCK", "releasing the glock");
            this.goLock = false;
            Log.d("POSITION", s);

            // make sure the robot keeps its head tilted up when it arrives
            this.tman.tilt("mid",6000);

            if (this.resettingInteraction) {
                this.resettingInteraction = false;
                this.finishReset();
            }

            /*
            Handle deciding whether movement behavior is finished
             */
            ArrayList<String> list = new ArrayList<String>();
            list.add(s);

            // does not matter the movement value
            this.ts.tellStateBehaviorDone("movement");
            /*
            Handle updating the environmental state
             */
            // must add current position to position state
            // before entering a new state, movement needs to be removed
            HashMap<String,ArrayList<String>> currEnvState = this.es.getEnvironmentalState();
            ArrayList<String> positionState = currEnvState.get("position");
            positionState.clear();
            positionState.add(s.toLowerCase());
            positionState.add("movement");
            Collections.sort(positionState);

            // update the debug view
            this.updateSenseView(this.es.toString());
        }
        if (s1.equals("aborted"))
            this.goLock = false;
    }

    /**
     * WAKE WORD LISTENER
     *
     * Change Temi's face from blue to orange or vice versa based on whether activated
     * by a wakeword
     */
    public void temiWakeWord(final boolean woke) {

        // change the UI
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                iv1.setImageURI(null);
                if (woke) {
                    iv1.setImageResource(R.drawable.temi_face_01__listening);
                }
                else {
                    iv1.setImageResource(R.drawable.temi_face_01__1_);
                }

            }
        });
    }

    /**
     *BEHAVIORS to EXECUTE
     */
    // say
    public void temiSay(String speech) {
        Log.d("EXECUTION","temi says " + speech);
        Log.d("SPEECH",this.sd.toString());

        String speech_cat = speech;

        if (speech.equals("greet"))
            speech = "Hello";
        else if (speech.equals("apologize"))
            speech = "I am sorry";
        else if (speech.equals("come_here"))
            speech = "Can you come here";
        else if (speech.equals("deny"))
            speech = "No";
        else if (speech.equals("excuse_self"))
            speech = "Excuse me";
        else if (speech.equals("farewell"))
            speech = "Goodbye";
        else if (speech.equals("follow_me"))
            speech = "Follow me";
        else if (speech.equals("go away"))
            speech = "Can you please leave me alone";
        else if (speech.equals("affirm"))
            speech = "Yes";
        else if (speech.equals("request_directions"))
            speech = "I am lost, can you help me?";
        else if (speech.equals("thank"))
            speech = "Thank you";
        else if (speech.equals("you_are_welcome"))
            speech = "You are welcome";

        else if (this.sd.inSpeechDb(speech)) {
            Log.d("STT","collecting unrecognized speech to say -- " + speech);
            Log.d("STT", this.sd.toString());
            speech = this.sd.getRandomPhrase(speech);
        }

        else
            speech = speech;
        TtsRequest ttsRequest = TtsRequest.create(speech, true);
        robot.speak(ttsRequest);
        ArrayList<String> temp = new ArrayList<String>();
        temp.add(speech_cat);

        // this behavior finishes immediately
        this.ts.tellStateBehaviorDone("speech",temp);
    }

    // cancel say
    public void temiQuiet() {
        Log.d("EXECUTION","temi stops speaking");
        robot.cancelAllTtsRequests();
    }

    // move
    public void temiGo(String location) {
        if (location.equals("true"))
            location = "default";
        setStatusText("goto: " + location);
        Log.d("EXECUTION", "temi is moving to " + location.toLowerCase());
        Log.d("avail locations", String.valueOf(robot.getLocations()));
        HashMap<String,ArrayList<String>> currEnvState = this.es.getEnvironmentalState();
        ArrayList<String> positionState = currEnvState.get("position");
        if (!positionState.contains("movement")) {
            positionState.add("movement");
        }
        Collections.sort(positionState);
        robot.goTo(location.toLowerCase());

        // this behavior finishes AFTER arriving at destination
    }

    // cancel move
    public void temiStop() {
        Log.d("EXECUTION", "temi is stopping");
        robot.stopMovement();
    }

    /**
     * Trigger freeze status.
     *
     * If frozen, Temi will not execute "self" behaviors. This is especially useful if a
     * wakeword is currently active, and Temi is listening for human speech. While listening,
     * the Temi should not trigger a state change.
     */
    public void freezeTrigger(boolean woke) {
        // handle the trigger freeze
        if (woke) {
            this.triggerFreeze = true;
            setStatusText("I'm listening");
        }
        else
            this.triggerFreeze = false;
            setStatusText("");
    }

    /**
     * Debugging stuff
     */
    public void addToHistory(String text) {
        this.history += (text + "\n");
    }

    public void updateStateView(final String text) {
        if (this.debug) {
            runOnUiThread(new Runnable() {

                @Override
                public void run() {

                    currExecutionStateView.setText(text);
                }
            });
        }
    }

    public void updateSenseView(final String text) {
        if (this.debug) {
            runOnUiThread(new Runnable() {

                @Override
                public void run() {

                    currSensingView.setText("SENSE:\n" + text.trim());
                }
            });
        }
    }

    public void updateTimerView(final String text) {
        if (this.debug) {
            runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    if (!(currTimerView.getText().equals("pointing") ||
                            currTimerView.getText().equals("tilting") ||
                            currTimerView.getText().equals("waiting")))
                        currTimerView.setText(text);
                }
            });
        }
    }

    public void displaySpeechOptions(final ArrayList<String> values) {
        return;
        /*
        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                Log.d("DISPLAY", String.valueOf(values));
                if (values.size() > 0)
                    btnView.setVisibility(View.VISIBLE);
                // add buttons programatically
                for (int i=0; i < values.size(); i++) {
                    Button btn = speechBtns.get(i);
                    btn.setVisibility(View.VISIBLE);
                    final String val = values.get(i);
                    btn.setText(values.get(i));
                    btn.setOnClickListener(new View.OnClickListener()
                    {
                        @Override
                        public void onClick(View v) {
                            temiListen(val);
                        }
                    });
                }
            }
        });

         */
    }

    public void removeSpeechOptions() {

        runOnUiThread(new Runnable() {

            @Override
            public void run() {
                Log.d("DISPLAY","gone");
                for (int i = 0; i < 12; i++) {
                    speechBtns.get(i).setVisibility(View.GONE);
                }
                btnView.setVisibility(View.GONE);
            }
        });
    }

    public void updateCurrOrientation(double angle, double timestamp) {
        Log.d("ORIENTATION UPDATED", String.valueOf(angle));
        this.orman.updateAngle(angle,timestamp);
    }

    @Override
    public void pointFinished() {
        this.ts.tellStateBehaviorDone("point");
        setStatusText("");

        // resume movement if paused
        this.movementPaused = false;
        if (this.movementPaused && this.pausedMovementDestination != null) {
            this.temiGo(this.pausedMovementDestination);
        }
        //robot.setDetectionModeOn(true);
    }

    @Override
    public void tiltFinished() {
        Log.d("TILT","tilt finished");
        this.ts.tellStateBehaviorDone("tilt");
        setStatusText("");
        //robot.setDetectionModeOn(true);
    }

    public void setStatusText(final String text) {
        runOnUiThread(new Runnable() {

            @Override
            public void run() {

                statusView.setText(text);
            }
        });
    }

    @Override
    public void waitFinished() {
        Log.d("WAIT","wait finished");
        this.ts.tellStateBehaviorDone("wait");
    }

    @Override
    public void onClick(View v) {
        if (v == this.startButton) {
            this.freezeTrigger(false);
            this.resetInteraction();
        }
        else if (v == this.endButton) {
            this.freezeTrigger(false);
            this.endInteraction();
        }
    }

    /*
    Other threads
    */
    public class DetectionTimer extends Thread {

        int state;
        boolean isActive;
        MainActivity main;
        int thread_id;

        public DetectionTimer(MainActivity main) {
            this.state = 4;
            this.isActive = true;
            this.main = main;
            // assign random thread id
            Random random = new Random();
            this.thread_id = random.nextInt(1000);
        }

        public void run(){

            this.main.setStatusText("I see you (3)");

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (!this.isActive)
                return;

            this.main.setStatusText("I see you (2)");

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (!this.isActive)
                return;

            this.main.setStatusText("I see you (1)");

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (!this.isActive)
                return;

            this.main.setStatusText("I see you (0)");

            this.main.triggerH1NearRobot();
            this.isActive = false;
        }
    }
}
