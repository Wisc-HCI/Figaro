# Figaro

### Required Setup

You can run the Figaro code without necessarily needing a lot of the hardware. As of May 2021, Figaro *requires* the following setup:

- An Ubuntu Linux computer. Figaro has been tested on Ubuntu 18.04.
- A microphone to capture speech
- ros1_bridge (place in the top-level Figaro directory with the other nodes): https://github.com/ros2/ros1_bridge
- cv_bridge (place in the top-level Figaro directory with the other nodes): https://github.com/ros-perception/vision_opencv

### Recommended Setup

Without the tabletop hardware, Figaro recommends the following additional components:

- A tablet for controlling Figaro

### For Full Replication

To fully replicate Figaro with the tabletop, we recommend the following additional hardware components:

- A Kinect V1 camera
- Two figurines constructed using the guidelines below (see "Figurine Construction")
- A tabletop constructed using the guidelines below (see "Tabletop Construction")
- A robot (currently the Temi robot)

### Dependencies

In the required setup, in addition to various python libraries, you will need:

- ROS2 (we used Eloquent)
- A Google Cloud credentials file. Place this in ```audio/audio```.
- mycroft-precise to detect wakewords. Follow the instructions on the mycroft github page to perform the *source install*. 

To replicate the full-setup, you will need:

- ROS1 for commmunicating with the Kinect V1 and the robot
- ros1_bridge for facilitating ROS2-ROS1 communication
- The sensor modules for the kinect: https://github.com/avin2/SensorKinect

## Tabletop Construction
(to be added soon)

## Figurine Construction
Each figurine is affixed to a joystick controller. Drill a hole into the center of the bottom of the figurine, such that it fits snuggly onto the joystick. The joystick base consists of the joystick, a small microcontroller with BLE capabilities, and a 3D-printed base to encapsulate the electronics. The microcontroller was programmed with [CircuitPython](https://circuitpython.org) using the [Adafruit_CircuitPython_BLE library](https://github.com/adafruit/Adafruit_CircuitPython_BLE). Code for the robot figurine can be found [here](https://github.com/Wisc-HCI/Figaro/blob/master/figure_gestures/microcontroller/microcontroller_code1.py).

(calibration instructions/code to be added)

The code for the figurines was designed to work with the following hardware:

- Adafruit Feather NRF52840 microcontroller: https://www.adafruit.com/product/40626
- Lithium Ion Polymer Ion Battery - 3.7V 100MAH: https://www.adafruit.com/product/1570
- Adafruit LiIon/LiPoly Backpack Add-On for Pro Trinket/ItsyBitsy: https://www.adafruit.com/product/2124
- Joystick Breakout Module: https://www.amazon.com/DEVMO-Joystick-Breakout-Controller-Arduino/dp/B07RB493PT/ref=sr_1_9?dchild=1&keywords=arduino+joystick&qid=1596144679&refinements=p_85%3A2470955011&rnid=2470954011&rps=1&sr=8-9


## Build Instructions (make sure to replace the "/path/to" with your own paths)

### Bare-minimum system without tabletop 

Build Figaro:
```
source /path/to/ros1/setup.bash
source /path/to/ros2/setup.bash
export ROS2_DISTRO=eloquent
cd /path/to/Figaro
colcon build --symlink-install --packages-skip ros1_bridge
```
### Replicating the full setup:

Build your ros1 workspace (where the SensorKinect packages have been placed)
```
cd /path/to/ros1_workspace && catkin_make
```

Build the ros bridge:
```
source /path/to/ros2/setup.bash
source /path/to/ros1/setup.bash
source /ros1/workspace/devel/setup.bash (commonly ~/catkin_ws/devel/setup.bash)
cd /path/to/Figaro
source install/setup.bash
export CMAKE_PREFIX_PATH=/path/to/Figaro/install/robot_msgs:/path/to/ros2:/path/to/ros1:/path/to/ros1_workspace/devel
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

## Run Instructions (we recommend each step being done in a separate terminal)

### Running the bare-minimum system without a tabletop or figurines

Each component needs to be run from within the directories of each package. 

Tablet server ($LOCAL_IP is the local ip address of your machine)
```
cd /path/to/Figaro/server/tablet
python3 tablet_server.py $LOCAL_IP
```

Ros2 nodes
```
ros2 run ctrl controller_node temi
ros2 run audio controller_node /path/to/mycroft-precise
ros2 run server controller_node $LOCAL_IP
ros2 run projector controller_node notest
```
### Replicating the full setup

Before running the ros2 nodes, start roscore in a new terminal
```
source /path/to/ros1/setup.bash
roscore
```

Before running the ros2 nodes, start the ros bridge in a new terminal
```
source /path/to/ros1/setup.bash
source /path/to/Figaro/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

Before running the ros2 nodes, start the camera in a new terminal
```
source /path/to/ros1/setup.bash
cd /ros1/workspace
source /ros1/workspace/devel/setup.bash
rosrun openni_camera openni_node
```
After the ros2 projector node has started, run the following other ros2 nodes. If the tabletop needs to be calibrated, set $CALIBRATE to "calibrate", otherwise "nocalibrate"
```
ros2 run figure_tracking controller_node $CALIBRATE
ros2 run touch_tracking controller_node $CALIBRATE
```

### Replicating Figaro with all hardware components

# Troubleshooting

- Errors may arise due to conflicts between sourcing ros1 and ros2. If this happens, we recommend building each Figaro and ros1_bridge in separate terminals as a common solution to this issue.
- We ran into issues running the Ros2 nodes whith ros1 in the library path. Within the terminals that you run the ros2 nodes, you can explicitly remove ros1 from the library path by setting the $LD_LIBRARY_PATH variable
