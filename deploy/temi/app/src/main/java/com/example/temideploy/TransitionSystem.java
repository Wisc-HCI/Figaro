package com.example.temideploy;

import android.content.Context;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class TransitionSystem {

    private Context context;
    private State init;
    ArrayList<State> states;
    private State curr;
    private MainActivity main;

    /*
    For interrupts
     */
    private State returnState;
    private String name;

    public TransitionSystem(Context context, MainActivity main) {
        this.context = context;
        this.states = new ArrayList<State>();
        this.init = null;
        this.curr = null;
        this.main = main;
        this.name = "";
    }

    public void reset() {
        this.curr = init;
    }

    public State getCurrState() {
        return this.curr;
    }

    public void setNextState(State next) {
        this.curr = next;
        this.main.updateStateView(this.curr.toStringAbridged());
    }

    public boolean executeCurr() {
        return this.curr.execute();
    }

    public void executeNext() {
        State next = this.curr.getNext();
        if (next != null) {
            this.setNextState(next);
            this.executeCurr();
        }
    }

    public void tellStateBehaviorDone(String beh, ArrayList<String> vals) {
        this.curr.behaviorFinished(beh,vals);
    }

    public void tellStateBehaviorDone(String beh) {
        this.curr.behaviorFinished(beh);
    }

    public State triggerChange(HashMap<String,ArrayList<String>> event, HashMap<String,ArrayList<String>> envState) {
        if (this.curr == null) {
            Log.d("WARNING","transition system not initialized and reset yet");
            return null;
        }

        State nextState = this.curr.step(event,envState);
        if (nextState == null) {
            Log.d("TRANSITION SYSTEM", "next state is null");
            return null;
        }

        Log.d("STATE CHANGE","state change triggered");
        this.curr.halt();
        // stop all behaviors (add all stop functions below)
        this.main.temiStop();
        this.setNextState(nextState);

        // if this TS has not run to completion
        if (this.executeCurr()) {
            return this.curr;
        }
        // else if this TS has run to completion
        else {

            // check if we can resume from an interrupt
            if (this.returnState != null) {
                State toReturn = this.returnState;
                this.relinquishReturnState();
                toReturn.resumeFromInterrupt();
            }
            else {
                Log.d("HISTORY",this.main.history);
            }

            // change init
            return null;
        }
    }

    public void loadTransitionSystemFile(String filename) {
        Log.d("LOADING","Filename is " + filename);

        try {
            InputStream is = context.getAssets().open(filename);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder;
            dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(is);

            this.loadTransitionSystem(doc);
        } catch (SAXException | ParserConfigurationException | IOException e1) {
            e1.printStackTrace();
        }
    }

    public void loadTransitionSystem(Document doc) {
        doc.getDocumentElement().normalize();
        System.out.println("Root element :" + doc.getDocumentElement().getNodeName());
        this.name = doc.getDocumentElement().getNodeName();

        NodeList stateList = doc.getElementsByTagName("state");
        NodeList transList = doc.getElementsByTagName("transition");

        for (int i = 0; i < stateList.getLength(); i++) {

            Node nNode = stateList.item(i);
            Element elem = (Element) nNode;
            String stateID = elem.getAttribute("id");

            HashMap<String,ArrayList<String>> behaviors = new HashMap<String,ArrayList<String>>();
            Element behaviorCollection = (Element) elem.getElementsByTagName("behaviors").item(0);
            NodeList behaviorList = behaviorCollection.getElementsByTagName("behavior");

            for (int j = 0; j < behaviorList.getLength(); j++) {
                Node behNode = behaviorList.item(j);
                Element behElem = (Element) behNode;
                String behCat = behElem.getAttribute("cat");

                ArrayList<String> vals = new ArrayList<String>();

                NodeList valList = behElem.getElementsByTagName("value");
                for (int k = 0; k < valList.getLength(); k++) {
                    Node valNode = valList.item(k);
                    Element valElem = (Element) valNode;
                    String valItem = valElem.getAttribute("val");
                    vals.add(valItem.toLowerCase());
                }

                if (!(behCat.equals("movement") && vals.size()==0))
                    behaviors.put(behCat,vals);
            }

            State newState = new State(stateID,behaviors,this.main);
            this.states.add(newState);
        }

        // determine the initial state
        Element initElem = (Element) doc.getElementsByTagName("init").item(0);
        int initID = Integer.parseInt(initElem.getAttribute("id"));
        for (int i = 0; i < states.size(); i++) {
            if (states.get(i).id == initID) {
                init = states.get(i);
                break;
            }
        }

        // handle the transitions
        for (int i = 0; i < transList.getLength(); i++) {
            Node nNode = transList.item(i);
            Element elem = (Element) nNode;
            int sourceID = Integer.parseInt(elem.getAttribute("source_id"));
            int targetID = Integer.parseInt(elem.getAttribute("target_id"));

            // find the actual source and targets
            State source = null;
            State target = null;
            for (int j = 0; j < states.size(); j ++) {
                State candidateState = states.get(j);
                if (candidateState.id == sourceID)
                    source = candidateState;
                if (candidateState.id == targetID)
                    target = candidateState;
            }
            // preconditions are that the source and target id's are valid
            HashMap<String,ArrayList<String>> events = new HashMap<String,ArrayList<String>>();
            Element eventCollection = (Element) elem.getElementsByTagName("events").item(0);
            NodeList eventList = eventCollection.getElementsByTagName("label");

            for (int j = 0; j < eventList.getLength(); j++) {
                Node eventNode = eventList.item(j);
                Element eventElem = (Element) eventNode;
                String eventCat = eventElem.getAttribute("label");

                ArrayList<String> vals = new ArrayList<String>();

                NodeList valList = eventElem.getElementsByTagName("val");
                for (int k = 0; k < valList.getLength(); k++) {
                    Node valNode = valList.item(k);
                    Element valElem = (Element) valNode;
                    String valItem = valElem.getAttribute("val");
                    vals.add(valItem.toLowerCase());
                }

                events.put(eventCat,vals);
            }

            HashMap<String,ArrayList<String>> envState = new HashMap<String,ArrayList<String>>();
            Element envCollection = (Element) elem.getElementsByTagName("env_state").item(0);
            NodeList envList = envCollection.getElementsByTagName("label");

            for (int j = 0; j < envList.getLength(); j++) {
                Node envNode = envList.item(j);
                Element envElem = (Element) envNode;
                String envCat = envElem.getAttribute("label");

                ArrayList<String> vals = new ArrayList<String>();

                NodeList valList = envElem.getElementsByTagName("val");
                for (int k = 0; k < valList.getLength(); k++) {
                    Node valNode = valList.item(k);
                    Element valElem = (Element) valNode;
                    String valItem = valElem.getAttribute("val");
                    vals.add(valItem.toLowerCase());
                }

                envState.put(envCat,vals);
            }

            Transition transition = new Transition(target,events,envState);
            source.outTrans.add(transition);

        }

        // enumerate states, checking if the only events they must listen for are "self"
        for (State st:this.states) {
            boolean fastSelfEvent = true;
            for (Transition trans:st.outTrans) {
                for (Map.Entry<String, ArrayList<String>> entry : trans.events.entrySet()) {
                    if (!entry.getKey().equals("")) {
                        fastSelfEvent = false;
                        break;
                    }
                }
                if (!fastSelfEvent)
                    break;
            }
            st.fastSelfEvent = fastSelfEvent;
        }

        Log.d("LOADING","Transition system loaded successfully.");
        Log.d("LOADING","\n" + this.toString());
    }

    /*
    Handle interrupts
     */
    public void setReturnState(State state) {
        this.returnState = state;
    }

    public void relinquishReturnState() {
        this.returnState = null;
    }

    public ArrayList<String> getInterruptValue() {
        /*
        Precondition is that there is a single transition emanating from the initial state
         */
        ArrayList<String> vals = new ArrayList<String>();

        HashMap<String, ArrayList<String>> events = this.init.outTrans.get(0).events;
        for (Map.Entry<String, ArrayList<String>> entry : events.entrySet()) {
            vals = entry.getValue();
        }

        return vals;
    }

    public String getInterruptName() {
        return this.name;
    }

    public String toString() {
        String toReturn = "";

        for (int i = 0; i < this.states.size(); i++) {
            toReturn += this.states.get(i).toString() + "\n";
        }

        return toReturn;
    }

}
