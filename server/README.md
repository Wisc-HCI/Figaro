#Server
Figaro/server runs a ROS2 node that also acts as a Flask server to provide two-way communication between the tablet interface and the rest of the Figaro system. The server node currently also controls the image projected to the tabletop interface. 

This README covers how to run the server and corresponding webpages without the ROS2 component.

## Dependencies
This Server node runs on Python3 and requires the following Python3 libraries:

1) flask
```pip install flask```

2) flask_cors
```pip install flask_cors```

3) flask_socketio
```pip install flask_socketio```

## Running
To run, you will need 2 terminals open.

### Termainal 1: Start the main server
In `Figaro\server\server` run:
```
python3 server_test.py 
```
NOTE: With the current setup, the server is live at localhost:5000, but this web address does not display anything.

### Terminal 2: Start the tablet server
In `Figaro\server\tablet` run:
```
python3 tablet_server.py
```
NOTE: Tablet interface is set to run at localhost:5010

