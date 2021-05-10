# Figaro

## General Setup

As of Feb. 2020, Figaro requires a setup with 4 distinct components -- 

- *Computer #1*: 1 Linux or OSX computer acting as the "controller," with which every other component communicates.
- *Computer #2*: 1 Linux computer acting as the "eyes," communicating directly with the camera(s) that detect the motion of the figurines. This computer also interfaces with the Robot.
- 1 Robot, currently the Pepper robot
- The tabletop hardware interface, with which users interact.

NOTE: to run Figaro, *only Computer #1 is absolutely required*. To run Figaro with the tabletop, both Computer #1 and Computer #2 are required. To deploy interactions on a robot, only Computer #2 is required.

## Dependencies

### Computer #1:
The controller computer is tested to run on ROS2 Eloquent. Install ROS2 Eloquent following the [ROS2 documentation](https://index.ros.org//doc/ros2/Installation/Eloquent/)

Also install colcon according to ROS2 documentation.

Finally install Stanford CoreNLP following the [CoreNLP documentation](https://stanfordnlp.github.io/CoreNLP/). Unzip the file into any directory.

### Computer #2: 
The camera/robot computer must have ROS2 Eloquent and ROS1 Melodic installed. ROS1 melodic can be installed following the [ROS1 Documentation](http://wiki.ros.org/melodic/Installation)

To interface with the Kinect V2 camera, the following must be done:

1) Install libfreenect2: https://github.com/OpenKinect/libfreenect2

2) Install pylibfreenect2: http://r9y9.github.io/pylibfreenect2/latest/installation.html

NOTE: you may have to set the shared library environment variable, which may look something like this
```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/freenect2/lib```

### In general, Figaro requires the following Python libraries:

1) PyAudio

2) google
```pip3 install --upgrade google-api-python-client```

3) google.cloud
```pip3 install --upgrade google-cloud-speech```
NOTE: You will also need a credential file for the Google Cloud service to work. Set the GOOGLE_APPLICATION_CREDENTIALS environment variable to the path the credential file is in.

4) PIL
pip3 install pillow

5) opencv-contrib
```pip3 install opencv-contrib-python```

NOTE: You need the contrib version. If you have the normal version already, you may need to uninstall it:
```
pip3 uninstall opencv-contrib-python
pip3 uninstall opencv-python
```

6) matplotlib

7) Z3 Solver
Install from the Z3 Solver Github instructions: https://github.com/Z3Prover/z3

8) flask

9) flask_cors

10) flask_socketio

11) pandas

12) Rasa NLU. To install RASA, you'll need to perform the following steps:

```
sudo python3 -m pip install rasa[spacy]
sudo python3 -m spacy download en_core_web_md
sudo python3 -m spacy link en_core_web_md en
```
13) nltk

14) mycroft-precise
Follow the instructions on the mycroft github page to perform the *source install*. 

## Build Instructions

On OSX, Figaro is currently only supported on OSX 10.15 -- previous versions of the OS may not work. On Linux, Figaro is currently only supported on Ubuntu 18.04. On either type of operating system, run the following:

### Computer #1

Computer #1 is currently responsible for running the following ros nodes:

- ctrl
- audio
- server
- projector

Additionally, at this moment of development Computer #1 must also run the tablet flask server and CoreNLP.

Run the following commands to build Figaro on Linux Computer #1:

```
cd <installation path>/Figaro
source ~/ros2_<distro>/install/setup.bash
rm -rf build/ install/ log/
```

On OSX, the following command should be run next:

```
colcon build --cmake-args ' -DCMAKE_CXX_COMPILER=/usr/bin/g++' ' -DCMAKE_C_COMPILER=/usr/bin/gcc'
```

The removal of the build and install folders seems critical to the compilation of custom messages for OSX. Note that on OSX, calling colcon build after the initial build appear to be faster if called from a new terminal window in which ROS2 has not been sourced. Regardless, the first time that this command is run, it could take a few minutes to complete.

On Linux, the following command should be run instead:

```
colcon build
```

NOTE: the ROS2 message packages robot_msgs and figaro_msgs need only be built once. If further changes to any other packages are made and need to be re-compiled, running the command ```sh build.sh``` will be sufficient. This shell script builds all other packages other than the messages.

### Computer #2

Computer #2 must be running Ubuntu 18.04. Computer #2 is currently responsible for running the following ROS nodes:

- figure_tracking
- ros1_bridge
- deploy_figaro (ROS 1 node)

The following steps will build the code

1) Turn the robot on and wait for it to start
2) Open up a new terminal window with 4 tabs
3) In the first tab, run ```make_ros1```
4) In the second tab, run ```build_figaro``` (it will complete with warnings, which is fine)
5) In the third tab, run ```build_bridge``` (it may complete with warnings, which is fine)
6) In the fourth tab, run ```run_roscore```

### Troubleshooting

- If ```colcon build``` is failing, try to delete these folders: `/Figaro/install`, `/Figaro/build`, `/Figaro/log`

## Instructions to run on Computer #1:

### 1) Open 5 new terminal windows, separate from the one that ```colcon build``` was run.

In terminal 1, change directories to /path/to/workspace/Figaro/ctrl/ctrl
In terminal 2, change directories to /path/to/workspace/Figaro/audio/audio
In terminal 3, change directories to /path/to/workspace/Figaro
In terminal 4, change directories to /path/to/workspace/Figaro/projector/projector
In terminal 5, change directories to /path/to/workspace/Figaro/server/tablet
In terminal 6, change directories to the Stanford CoreNLP directory 

### 2) In terminals 1-6, source ROS and your workspace:

On Linux and OSX, this looks something like:

```
source ~/ros2_eloquent/install/setup.bash
source /path/to/workspace/install/setup.bash
```

### 3) Run the program

In terminal 1, run ```ros2 run ctrl controller_node```
In terminal 2, run ```ros2 run audio controller_node <path_to_mycroft>```
In terminal 3, run ```ros2 run server controller_node <ip_address>```
In terminal 4, run ```ros2 run projector controller_node```
In terminal 5, run ```python3 tablet_server.py <ip_address>```
In terminal 6, run: ```java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000```

### 5) Wait for the projector UI to show up

This usually takes about 20 seconds

### 6) Open the following URL on a browser:

```
http://localhost:5010/index
```

## Instructions to run figure tracking on Computer #2:

### 1) Open 1 new terminal window, separate from the one that ```colcon build``` (or ```sh build.sh``` if the messages did not need to be built) was run.

### 2) Run ```rosrun openni_camera openni_node``` to start the ros1 Kinect node

### 3) Run ```bash run_tracker``` to start the figure tracker

### 4) Run ```bash run_touch_tracker``` to start the touch tracking node

### 3) Calibrate the camera

In the same terminal window, type ```pos```. Then on the window titled "ir", click the four corners of the projected image in the following order: top left, top right, bottom right, bottom left. This tells Figar where the projected interface is located.
 
## Instructions for running the robot using Computer #2

Open 2 new tabs on computer #2. Follow the steps below to begin running the robot code:

1) In the fifth tab, run ```start_bridge```
2) In the sixth tab, run ```start_robot```
