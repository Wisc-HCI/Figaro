package com.example.temideploy;

import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Transition {

    State target;
    HashMap<String, ArrayList<String>> events;
    HashMap<String, ArrayList<String>> envState;

    public Transition(State target, HashMap events, HashMap envState) {
        this.target = target;
        this.events = events;
        this.envState = envState;
    }

    public State step(HashMap<String,ArrayList<String>> event,HashMap<String,ArrayList<String>> currEnvState) {
        State match = null;
        Log.d("TRANSITION","avail event is" + String.valueOf(this.events));
        Log.d("TRANSITION", "submitted event is " + event);
        if (event.equals(this.events)) {
            Log.d("TRANSITION","event is a match");
            // now test to see whether the environment that is specific by the transition
            // matches the currently recorded environment
            // NOTE: for Temi, the criteria for a match is that the currently-sensed environment
            //       must contain every element in the transition's env state
            boolean isMatch = true;
            for (Map.Entry<String, ArrayList<String>> entry : this.envState.entrySet()) {
                String label = entry.getKey();
                ArrayList<String> value = entry.getValue();

                for (String val : value) {
                    if (!currEnvState.get(label).contains(val)) {
                        Log.d("TRANSITION",currEnvState.get(label) + "!=" + label);
                        isMatch = false;
                        break;
                    }
                }
                //if (!currEnvState.get(label).equals(value)) {
                //    Log.d("TRANSITION",currEnvState.get(label) + "!=" + label);
                //    isMatch = false;
                //}
            }

            if (isMatch)
                match = this.target;
        }

        return match;
    }

    public int getScore(HashMap<String,ArrayList<String>> currEnvState) {
        /*
        PRECONDITION: the transition already matches the event and currEnvState
         */
        int score = 0;

        for (Map.Entry<String,ArrayList<String>> entry : currEnvState.entrySet()) {
            String key = entry.getKey();
            ArrayList<String> val = entry.getValue();

            if (this.envState.containsKey(key)) {
                ArrayList<String> transVals = this.envState.get(key);

                for (String v : val) {
                    if (transVals.contains(v))
                        score++;
                }
            }
        }

        return score;
    }

    public String oneLineToString() {
        String toReturn = "  --> ( events: ";
        for (Map.Entry<String, ArrayList<String>> entry : this.events.entrySet()) {
            toReturn += " " + entry.getKey() + "=" + entry.getValue();
        }
        toReturn += " ) + ( env state: ";
        for (Map.Entry<String, ArrayList<String>> entry : this.envState.entrySet()) {
            toReturn += " " + entry.getKey() + "=" + entry.getValue();
        }

        return toReturn;
    }

    public String toString() {
        String toReturn = "\n  TRANSITION";
        toReturn += "\n    target_id: " + this.target.id;
        toReturn += "\n    events: ";
        for (Map.Entry<String, ArrayList<String>> entry : this.events.entrySet()) {
            toReturn += "\n      " + entry.getKey() + " = " + entry.getValue();
        }
        toReturn += "\n    environmental state: ";
        for (Map.Entry<String, ArrayList<String>> entry : this.envState.entrySet()) {
            toReturn += "\n      " + entry.getKey() + " = " + entry.getValue();
        }
        return toReturn;
    }

}
