package com.example.temideploy;

import android.util.Log;

import com.robotemi.sdk.Robot;

import java.util.ArrayList;

public class WaitManager {

    // observers
    private ArrayList<WaitObserver> wObservers;

    // robot
    private Robot robot;

    public WaitManager(Robot robot) {
        // observers
        wObservers = new ArrayList<WaitObserver>();
        this.robot = robot;
    }

    public void registerObserver(WaitObserver waitObserver) {
        this.wObservers.add(waitObserver);
    }

    public void removeObserver(TiltObserver tiltObserver) {
        this.wObservers.remove(tiltObserver);
    }

    public void notifyObservers() {
        for (WaitObserver observer: wObservers) {
            Log.d("TILT","notifying observers");
            observer.waitFinished();
        }
    }

    public class WaitThread extends Thread {

        private long ms;
        private Robot robot;
        private WaitManager manager;

        public WaitThread(long ms, Robot robot, WaitManager manager) {
            this.ms = ms;
            this.robot = robot;
            this.manager = manager;
        }

        public void run(){

            Log.d("TILT","initiating wait");
            try {
                Thread.sleep(this.ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Log.d("TILT","ended wait");
            this.manager.notifyObservers();

        }

    }

}