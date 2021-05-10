package com.example.temideploy;

import android.util.Log;

import com.robotemi.sdk.Robot;

import java.util.ArrayList;

public class PointManager {

    // observers
    private ArrayList<PointObserver> pObservers;

    // robot
    private Robot robot;

    public PointManager(Robot robot) {
        // observers
        pObservers = new ArrayList<PointObserver>();
        this.robot = robot;
    }

    public void registerObserver(PointObserver pointObserver) {
        this.pObservers.add(pointObserver);
    }

    public void removeObserver(PointObserver pointObserver) {
        this.pObservers.remove(pointObserver);
    }

    public void point(double currOrientation, double direction) {
        PointThread tt = new PointThread(currOrientation, direction, this.robot, this);
        tt.start();
    }

    public void notifyObservers() {
        for (PointObserver observer: pObservers) {
            observer.pointFinished();
        }
    }

    public class PointThread extends Thread {

        private int currOrientation;
        private int direction;
        private Robot robot;
        private PointManager manager;

        public PointThread(double currOrientation, double direction, Robot robot, PointManager manager) {
            this.currOrientation = (int) Math.round(currOrientation);
            this.direction = (int) Math.round(direction);
            this.robot = robot;
            this.manager = manager;
        }

        public void run(){

            // figure out how much to turn by
            int return_angle = 0;
            int target_angle = 0;

            // if assuming temi will always be a certain angle when at a destination
            return_angle = direction;
            target_angle = -1 * direction;

            /*
            // if we are actively tracking temi's orientation
            // calculate left turn
            int ltarget = this.direction - 360;
            int left = this.currOrientation - ltarget;

            // calculate right turn
            int rtarget = this.direction;
            int right = rtarget - this.currOrientation;

            if (left < right) {
                target_angle = left;
                return_angle = -1*left;
            } else {
                target_angle = -1*right;
                return_angle = right;
            }
            */

            Log.d("ANGLES",target_angle + "   " + return_angle);

            this.robot.turnBy(target_angle);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            this.robot.tiltAngle(25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            this.robot.tiltAngle(-25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            this.robot.tiltAngle(0);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            this.robot.turnBy(return_angle);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            this.manager.notifyObservers();

        }

    }

}
