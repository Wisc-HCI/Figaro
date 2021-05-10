package com.example.temideploy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class EnvironmentalState {

    private HashMap<String, ArrayList<String>> currState;

    public EnvironmentalState(ArrayList<String> envCategories) {
        this.currState = new HashMap<String,ArrayList<String>>();
        for (String cat : envCategories) {
            currState.put(cat,new ArrayList<String>());
        }
    }

    public void putState(String cat, ArrayList<String> vals) {
        this.currState.put(cat,vals);
    }

    public HashMap<String,ArrayList<String>> getEnvironmentalState(){
        return this.currState;
    }

    public String toString() {
        String toReturn = "\n";
        for (Map.Entry<String, ArrayList<String>> entry : this.currState.entrySet()) {
            toReturn += "\n    " + entry.getKey() + " = " + entry.getValue();
        }
        return toReturn;
    }

}
