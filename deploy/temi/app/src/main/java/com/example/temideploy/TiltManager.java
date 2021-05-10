package com.example.temideploy;

import android.util.Log;

import com.robotemi.sdk.Robot;

import java.util.ArrayList;

public class TiltManager {

    // observers
    private ArrayList<TiltObserver> tObservers;

    // robot
    private Robot robot;

    // tilt state
    private String tiltState;

    public TiltManager(Robot robot) {
        // observers
        tObservers = new ArrayList<TiltObserver>();
        this.robot = robot;
        this.tiltState = "mid";
    }

    public void tilt(String category,long ms) {
        switch (category) {
            case "up":
                this.tiltState = "up";
                this.tilt(55,ms);
                break;
            case "down":
                this.tiltState = "down";
                this.tilt(-25,ms);
                break;
            case "mid":
                this.tiltState = "mid";
                this.tilt(0,ms);
                break;
            default:
                break;
        }
    }

    public void tilt(int angle,long ms) {
        if (ms < 1000) {
            ms = 1000;
        }

        TiltThread tt = new TiltThread(angle, ms, this.robot, this);
        tt.start();
    }

    public void maintainTiltAngle() {
        switch (this.tiltState) {
            case "up":
                this.tilt(55,6000);
                break;
            case "down":
                this.tilt(-25,6000);
                break;
            case "mid":
                this.tilt(0,6000);
                break;
            default:
                break;
        }
    }

    public void registerObserver(TiltObserver tiltObserver) {
        this.tObservers.add(tiltObserver);
    }

    public void removeObserver(TiltObserver tiltObserver) {
        this.tObservers.remove(tiltObserver);
    }

    public void notifyObservers() {
        for (TiltObserver observer: tObservers) {
            Log.d("TILT","notifying observers");
            observer.tiltFinished();
        }
    }

    public class TiltThread extends Thread {

        private int angle;
        private long ms;
        private Robot robot;
        private TiltManager manager;

        public TiltThread(int angle, long ms, Robot robot, TiltManager manager) {
            this.angle = angle;
            this.ms = ms;
            this.robot = robot;
            this.manager = manager;
        }

        public void run(){

            Log.d("TILT","initiating tilt");
            this.robot.tiltAngle(this.angle);
            try {
                Thread.sleep(this.ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            this.robot.tiltAngle(0);
            Log.d("TILT","ended tilt");
            this.manager.notifyObservers();

        }

    }

}
