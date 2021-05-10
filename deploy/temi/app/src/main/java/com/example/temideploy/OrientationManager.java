package com.example.temideploy;

import android.util.Log;

import com.robotemi.sdk.Robot;

import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgcodecs.Imgcodecs.imread;

// this is meant to be an observable
public class OrientationManager {

    private CaptureThread caprun;
    private CameraController cam;
    //private CaptureThread ct;
    private MainActivity main;
    private Dictionary dictionary;

    // observers
    //private ArrayList<OrientationObserver> oObservers;

    private double orientation;
    private double mostRecentTime;

    public OrientationManager(Robot robot,CameraController cam, MainActivity main) {
        this.cam = cam;
        this.main = main;
        this.caprun = new CaptureThread(cam,main);
        //this.ct = new CaptureThread(this.cam);
        this.caprun.start();

        // observers
        //oObservers = new ArrayList<OrientationObserver>();

        // arucos
        if (!OpenCVLoader.initDebug())
            Log.e("OpenCv", "Unable to load OpenCV");
        else
            Log.d("OpenCv", "OpenCV loaded");

        this.dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_50);
        this.orientation = 0.0;
        this.mostRecentTime = -1.0;
    }

    public void updateAngle(double angle, double timestamp) {
        if (timestamp > this.mostRecentTime) {
            this.mostRecentTime = timestamp;
            this.orientation = angle;
        }
    }

    public double getCurrOrientation() {
        return this.orientation;
    }

    public class CaptureThread extends Thread {

        private CameraController cam;

        public CaptureThread(CameraController cam,MainActivity main) {
            this.cam = cam;
        }

        public void run(){
            this.cam.openCamera();
            int no_detect_seconds = 2;
            int iter_time = 500;
            int no_detect_iterations = (no_detect_seconds*1000)/iter_time;
            int no_detect_count = 0;
            int curr_detection_state = 0;
            while (!Thread.interrupted()) {

                try {
                    Thread.sleep(500);
                    String path = this.cam.takePicture();
                    //Log.d("CAM", "picture successfully taken and stored at " + path);

                    // now read the image in opencv and perform aruco detection
                    Mat ids = new Mat();
                    Mat mat1 = imread(path);
                    List<Mat> corners = new ArrayList<>();
                    Aruco.detectMarkers(mat1, dictionary, corners, ids);
                    if (corners.size()>0) {
                        //Log.d("OPENCV","aruco detected");
                        no_detect_count = 0;
                        if (curr_detection_state != 2) {
                            curr_detection_state = 2;
                            main.onDetectionStateChanged(2);
                        }
                    }
                    else {
                        no_detect_count += 1;
                        if (no_detect_count > no_detect_iterations)
                            if (curr_detection_state != 0) {
                                curr_detection_state = 0;
                                main.onDetectionStateChanged(0);
                            }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            this.cam.closeCamera();

        }

    }

}
