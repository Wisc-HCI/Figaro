package com.example.temideploy;

import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class State {

    ArrayList<Transition> outTrans;
    HashMap<String,ArrayList<String>> behaviors;
    HashMap<String,HashMap<ArrayList<String>,TransitionSystem>> interrupts;
    int id;
    boolean fastSelfEvent;
    ArrayList<String> behaviorsDone;
    TimerThread tt;
    MainActivity main;

    public State(String id, HashMap behaviors, MainActivity main) {
        this.id = Integer.parseInt(id);
        this.behaviors = behaviors;
        this.outTrans = new ArrayList<Transition>();
        this.fastSelfEvent = true;
        this.behaviorsDone = new ArrayList<String>();
        this.tt = null;
        this.main = main;
    }

    public void addInterrupts(HashMap<String,HashMap<ArrayList<String>,TransitionSystem>> interrupts) {
        this.interrupts = interrupts;
    }

    public State step(HashMap<String,ArrayList<String>> event,HashMap<String,ArrayList<String>> envState) {
        /*
        First determine if an interrupt satisfied this event
         */
        TransitionSystem interrupt = null;

        // check if we are moving
        boolean moving = false;
        if (this.behaviors.containsKey("movement") && (!this.behaviorsDone.contains("movement") && this.behaviors.get("movement").size() > 0)) {
            Log.d("STATE","cannot look at interrupts because robot is moving");
            moving = true;
        }

        // do not interrupt if moving
        if (!moving) {
            Log.d("STATE","we will try searching for interrupts");
            for (Map.Entry<String, ArrayList<String>> entry : event.entrySet()) {
                String cat = entry.getKey();
                ArrayList<String> vals = entry.getValue();
                Log.d("STATE", "searching for interrupts:::(see below for what searching for)");
                Log.d("STATE",cat);
                //Log.d("STATE", String.valueOf(vals));
                //Log.d("STATE", String.valueOf(this.interrupts));
                if (this.interrupts.containsKey(cat)) {
                    if (this.interrupts.get(cat).containsKey(vals))
                        Log.d("STATE",cat + " is an interrupt that could happen");
                        interrupt = this.interrupts.get(cat).get(vals);
                }
            }
        }

        HashMap<State,Integer> matchScores = new HashMap<State,Integer>();
        State match = null;
        Log.d("STATE (out trans)", String.valueOf(this.outTrans));
        for (Transition trans:this.outTrans) {
            Log.d("STATE", "looking at transition");
            State nextState = trans.step(event,envState);
            if (nextState != null) {

                // we will go with this match
                // add to the history
                this.main.addToHistory(trans.oneLineToString());

                matchScores.put(nextState,trans.getScore(envState));
                //match = nextState;
                //break;
            }
        }

        // pick the best match
        int bestScore = -1;
        for (Map.Entry<State, Integer> entry : matchScores.entrySet()) {
            int currScore = entry.getValue();
            if (currScore > bestScore) {
                bestScore = currScore;
                match = entry.getKey();
            }
        }

        // choose whether to pursue match, interrupt, or none
        if (match == null && interrupt != null) {
            // we are going with an interrupt
            // add to the history
            this.main.addToHistory("interrupt started");

            // begin the interrupt
            Log.d("STATE","Beginning interrupt");
            this.beginInterrupt(interrupt);
            return null;
        }
        else if (match != null) {
            return match;
        }
        else {
            return null;
        }
    }

    public boolean execute() {
        this.behaviorsDone.clear();
        Log.d("EXECUTION", "executing the following state");
        Log.d("EXECUTION",this.toString());

        // add to the history
        this.main.addToHistory(this.id + "");

        // if no behaviors to execute....
        boolean behaviorsToExecute = false;
        boolean more = true;
        for (Map.Entry<String, ArrayList<String>> entry : this.behaviors.entrySet()) {
            ArrayList<String> value = entry.getValue();
            String beh = entry.getKey();
            if (beh.equals("sys")) {
                if (value.contains("off")) {
                    more = false;
                }
            }
            if (value.size() > 0)
                behaviorsToExecute = true;
            break;
        }

        // if nothing to execute
        if (!behaviorsToExecute) {

            // if there is a self-directed transition
            boolean containsSelf = this.testIfSelfTransExists();
            if (containsSelf)
                this.initiateTimer();
            // send options for the human for how to talk to the robot
            this.visualizeSpeechOptions();
        }
        return more;
    }

    public void visualizeSpeechOptions() {
        // get list of responses that the human can provide to the robot
        ArrayList<String> responses = new ArrayList<String>();

        // do the main transition
        for (Transition trans : this.outTrans) {
            HashMap<String, ArrayList<String>> events = trans.events;
            for (Map.Entry<String, ArrayList<String>> entry : events.entrySet()) {
                if (entry.getKey().equals("h1_speech")) {
                    ArrayList<String> values = entry.getValue();
                    for (String val : values) {
                        if (!responses.contains(val)) {
                            responses.add(val);
                        }
                    }
                }
            }
        }

        // do the interrupts
        for (Map.Entry<String,HashMap<ArrayList<String>,TransitionSystem>> entry : this.interrupts.entrySet()) {
            if (entry.getKey().equals("h1_speech")) {
                HashMap<ArrayList<String>, TransitionSystem> values = entry.getValue();
                for (Map.Entry<ArrayList<String>,TransitionSystem> secondEntry: values.entrySet()) {
                    ArrayList<String> speeches = secondEntry.getKey();
                    for (String val : speeches) {
                        if (!responses.contains(val)) {
                            responses.add(val);
                        }
                    }
                }
            }
        }

        Log.d("STATE RESPONSE VIS", String.valueOf(responses));

        this.main.displaySpeechOptions(responses);
    }

    private boolean testIfSelfTransExists() {
        boolean containsSelf = false;
        for (Transition trans:this.outTrans) {
            if (trans.events.containsKey(""))
                containsSelf = true;
        }

        return containsSelf;
    }

    public State getNext() {
        if (outTrans.size() > 0) {
            Transition t = outTrans.get(0);
            State next = t.target;
            return next;
        }
        return null;
    }

    public boolean checkIfAllBehaviorsDone() {
        boolean allBehsFinished = true;
        for (Map.Entry<String, ArrayList<String>> entry : this.behaviors.entrySet()) {
            if (!this.behaviorsDone.contains(entry.getKey()))
                allBehsFinished = false;
        }
        return allBehsFinished;
    }

    public void behaviorFinished(String behCat, ArrayList<String> beh) {
        Log.d("TERMINATING BEHAVIOR", behCat + " -- " + beh);
        Log.d("AVAILABLE TO TERMINATE", String.valueOf(this.behaviors));
        if (!this.behaviors.get(behCat).equals(beh))
            return;
        this.behaviorsDone.add(behCat);
        boolean allBehsFinished = this.checkIfAllBehaviorsDone();
        if (allBehsFinished) {
            Log.d("STATE","behaviors done starting timer");
            // if there is a self-directed transition
            boolean containsSelf = this.testIfSelfTransExists();
            if (containsSelf)
                this.initiateTimer();
            // send options for the human for how to talk to the robot
            this.visualizeSpeechOptions();
        }
    }

    public void behaviorFinished(String behCat) {
        this.behaviorsDone.add(behCat);
        boolean allBehsFinished = this.checkIfAllBehaviorsDone();
        if (allBehsFinished) {
            Log.d("STATE","behaviors done starting timer");
            // if there is a self-directed transition
            boolean containsSelf = this.testIfSelfTransExists();
            if (containsSelf)
                this.initiateTimer();
            // send options for the human for how to talk to the robot
            this.visualizeSpeechOptions();
        }
    }

    private void initiateTimer() {
        this.visualizeSpeechOptions();
        // halt in case another thread is currently going
        this.halt();

        // see if contains wait
        boolean containsWait = false;
        if (this.behaviors.containsKey("wait")) {
            Log.d("TIMER","LONG TIMER STARTING");
            containsWait = true;
        }

        int initialSleepTime = 4000;
        // if we have a wait behavior
        if (containsWait)
            initialSleepTime = 30000;
        else if (!this.fastSelfEvent)
            initialSleepTime = 10000;
        this.tt = new TimerThread(this.main,initialSleepTime);
        this.tt.start();
    }

    public void halt() {
        // kill the thread
        if (this.tt != null) {
            this.tt.isActive = false;
        }
    }

    /*
    Handle interrupts
     */
    public void beginInterrupt(TransitionSystem interrupt) {
        // halt the timer if necessary
        halt();

        // reset the interrupt TS
        interrupt.reset();

        // set the return state for the interrupt TS
        interrupt.setReturnState(this);

        // halt all currently-executing behaviors
        main.temiStop();
        main.temiQuiet();

        // store unfinished behaviors in a special datastructure
        // unneeded -- behaviors done stores it for us

        main.switchTs(interrupt);
        interrupt.executeNext();
        State curr = interrupt.getCurrState();
        main.prepareStateExecution(curr);
    }

    public void resumeFromInterrupt() {
        // pass state to main
        main.resumeTs();
        main.executeState(this);

        // start the timer if no behaviors left
        if (checkIfAllBehaviorsDone()) {
            boolean containsSelf = this.testIfSelfTransExists();
            if (containsSelf)
                this.initiateTimer();
        }

        // temporary code
        // see what behaviors are getting resumed

    }

    public String toString() {
        String toReturn = this.toStringAbridged();

        for (int i = 0; i < this.outTrans.size(); i++) {
            toReturn += this.outTrans.get(i);
        }
        return toReturn;
    }

    public String toStringAbridged() {
        String toReturn = "STATE";
        toReturn += "\n  id: " + this.id;
        toReturn += "\n  fse: " + this.fastSelfEvent;
        toReturn += "\n  num out trans: " + this.outTrans.size();
        toReturn += "\n  behaviors: ";
        for (Map.Entry<String, ArrayList<String>> entry : behaviors.entrySet()) {
            toReturn += "\n    " + entry.getKey() + " = " + entry.getValue();
        }

        return toReturn;
    }

    /*
Other threads
 */
    public class TimerThread extends Thread {

        boolean isActive;
        MainActivity main;
        int initialSleepTime;
        int thread_id;

        public TimerThread(MainActivity main,int initialSleepTime) {
            this.isActive = true;
            this.main = main;
            this.initialSleepTime = initialSleepTime;
            // assign random thread id
            Random random = new Random();
            this.thread_id = random.nextInt(1000);
        }

        public void run(){

            while (this.initialSleepTime > 0) {
                this.main.updateTimerView("...waiting " + (this.initialSleepTime/1000) + " seconds to trigger self... (id=" + this.thread_id + ")");
                if (this.isActive)
                    Log.d("TIMER","WAITING");
                // the initial sleep
                try {
                    Thread.sleep(1000);
                    this.initialSleepTime -= 1000;

                    if (this.initialSleepTime < 10000 && this.isActive) {
                        this.main.setStatusText("moving on in " + this.initialSleepTime/1000 + " seconds");
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            if (this.isActive) {
                this.main.updateTimerView("initial trigger");
                triggerStateChange();
            }

            while (this.isActive) {
                try {
                    Thread.sleep(1000);
                    if (!this.isActive)
                        break;
                    Log.d("TIMER","fires");
                    this.main.updateTimerView("attempting to trigger self");
                    triggerStateChange();

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            this.main.updateTimerView("");
        }

        public void triggerStateChange() {
            HashMap<String,ArrayList<String>> event = new HashMap<String,ArrayList<String>>();
            ArrayList<String> eventVal = new ArrayList<String>();
            eventVal.add("self");
            event.put("",eventVal);
            this.main.triggerStateChange(event);
        }
    }
}
