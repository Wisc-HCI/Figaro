# Figaro

### Required Setup

You can run the Figaro code without necessarily needing a lot of the hardware. As of May 2021, Figaro *requires* the following setup:

- An Ubuntu Linux computer. Figaro has been tested on Ubuntu 18.04.
- A microphone to capture speech

### Recommended Setup

Without the tabletop hardware, Figaro recommends the following additional components:

- A tablet for controlling Figaro

### For Full Replication

To fully replicate Figaro with the tabletop, we recommend the following additional components:

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
(to be added soon)

## Build Instructions
(to be added soon)

## Run Instructions
(to be added soon)
