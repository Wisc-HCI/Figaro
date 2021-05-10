import time
import math
import signal
import sys
import json
import random

# google cloud stuff (pub sub)
from google.oauth2 import service_account
from google.cloud import pubsub_v1

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

class Gesture_Publisher_Node:
    def __init__(self, project_id, topic_name, act):
        # setup ble connection
        self.ble = BLERadio()

        # set 'zero' values for when joystick is in middle
        self.x0 = 32.4
        self.y0 = 33.3

        # set the actor as human or robot
        self.actor = act

        # Authenticate google cloud services
        pubsub_key = json.load(open('pubsub_key.json'))
        credentials = (service_account.Credentials.from_service_account_info(pubsub_key))


        # Initialize a Publisher client.
        self.client = pubsub_v1.PublisherClient(credentials=credentials)

        # Create a fully qualified identifier in the form of
        # `projects/{project_id}/topics/{topic_id}`
        self.topic_path = self.client.topic_path('verification-of-hri', 'robot_topic')

        # Create the topic
        self.topic = self.client.create_topic(self.topic_path)


    def get_callback(self, api_future, data, ref):
        """Wrap message data in the context of the callback function."""

        def callback(api_future):
            try:
                '''
                print(
                    "Published message {} now has message ID {}".format(
                        data, api_future.result()
                    )
                )
                '''
                ref["num_messages"] += 1
            except Exception:
                print(
                    "A problem occurred when publishing {}: {}\n".format(
                        data, api_future.exception()
                    )
                )
                raise

        return callback

    def test_wait(self):
        # Keep track of the number of published messages.
        print("testing wait")
        ref = dict({"num_messages": 0})

        wait_time = 0.1
        mag = 0
        direct = 0
        btn = 0
        upward = True

        while True:

            start_time = time.time()
            while True:
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                end_time = time.time()

                #print('sending ' + data)
                data = bytes(data, 'UTF-8')
                
                #print(data)
                print("-------")
                print(data)
                try:
                    #print("sending to google cloud")
                    api_future = self.client.publish(self.topic_path, data)
                    api_future.add_done_callback(self.get_callback(api_future, data, ref))
                    #print("sent to google cloud")
                except:
                    print("could not send to google cloud")

                time.sleep(0.1)

                if end_time - start_time > wait_time:
                    print("NEW TIME")
                    break

            if btn == 0:
                btn = 50
            else:
                btn = 0

            if wait_time > 3:
                upward = False
            if wait_time < 0.1:
                upward = True
            if not upward:
                wait_time = wait_time/2.0
            else:
                wait_time = wait_time*2.0

    def publish_helper(self, data, ref):
        data = bytes(data, 'UTF-8')
        print("-------")
        print(data)
        try:
            #print("sending to google cloud")
            api_future = self.client.publish(self.topic_path, data)
            api_future.add_done_callback(self.get_callback(api_future, data, ref))
            #print("sent to google cloud")
        except:
            print("could not send to google cloud")

    def test_tilt(self):
        # Keep track of the number of published messages.
        ref = dict({"num_messages": 0})

        wait_time = 0.1
        mag = 0
        direct = 0
        btn = 50
        upward = True

        while True:

            # tilt can start by participants first pushing down and then moving
            timestamp = time.time()
            data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
            self.publish_helper(data,ref)
            time.sleep(0.1)

            btn=0
            direct = 0
            mag = 0
            while mag < 10000:
                # tilt can start by participants first pushing down and then moving
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                time.sleep(0.1)
                mag += 750

            for i in range(20):
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                time.sleep(0.1)

            btn = 50
            timestamp = time.time()
            data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
            self.publish_helper(data,ref)
            time.sleep(0.1)

            while mag > 0:
                # tilt can start by participants first pushing down and then moving
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                time.sleep(0.1)
                mag -= 3000

            mag = 0
            for i in range(40):
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                time.sleep(0.1)

            # tilt can also start by participants moving, and then pushing

    def test_point(self):
        # Keep track of the number of published messages.
        ref = dict({"num_messages": 0})

        wait_time = 0.1
        mag = 0
        direct = 0
        btn = 50

        while True:

            # start the point from a random direction
            direct = random.randrange(0,360)
            if direct > 180:
                direct = direct - 360

            while mag < 12000:
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                mag += 700
                time.sleep(0.1)

            for i in range(20):
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                time.sleep(0.1)

            # end the point
            while mag > 0:
                timestamp = time.time()
                data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(btn)
                self.publish_helper(data,ref)
                mag -= 1100
                time.sleep(0.1)
            mag = 0

    def test_pub(self):
        # Keep track of the number of published messages.
        ref = dict({"num_messages": 0})

        offset_angle = 0
        offset_magnitude = 0
        high_button = 0
        calibrate = True
        calib_mtx_angle = []
        calib_mtx_mag = []
        calib_mtx_btn = []
        begin_calibrate = True
        # When you publish a message, the client returns a future.

        direct = 0 # 0-359
        actual_direct = 0 # goes from 0-180, 0-(-180)
        mag = 0 # goes from -10000-10000
        btn = 50 # goes from 0-50

        mode = None # wait, tilt, point
        wait_start = None
        while(True):
            #interp = classify(xVal, yVal, bVal)

            if mode is None:
                mode = "wait"
                btn = 0
                mag = 0
                wait_start = time.time()
            elif mode == "wait" and time.time() - wait_start > 3:
                mode = "tilt"
                btn = 0
                mag = 10000
                direct = 0
                wait_start = None
            elif mode == "tilt" and direct == 180:
                btn = 50
                direct = 0
                mag = 10000
                actual_direct = direct
                mode = "point"
            elif mode == "point" and actual_direct > -15 and actual_direct < 0:
                mode = None

            elif mode == "tilt":
                direct += 10
                actual_direct = direct
            elif mode == "point":
                direct += 10
                actual_direct = direct
                if actual_direct > 180:
                    actual_direct = actual_direct - 360

            timestamp = time.time()
            data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(actual_direct) + "#" + str(btn)
            print(mode)
            print(data)
            
            time.sleep(0.2)

            #print('sending ' + data)
            data = bytes(data, 'UTF-8')
            
            #print(data)
            print("-------")
            try:
                #print("sending to google cloud")
                api_future = self.client.publish(self.topic_path, data)
                api_future.add_done_callback(self.get_callback(api_future, data, ref))
                #print("sent to google cloud")
            except:
                print("could not send to google cloud")

    def pub(self):
        # Keep track of the number of published messages.
        ref = dict({"num_messages": 0})

        offset_angle = 0
        offset_magnitude = 0
        high_button = 0
        calibrate_angle = True
        calibrate_still = True
        calib_mtx_angle = []
        calib_mtx_mag = []
        calib_mtx_btn = []
        begin_calibrate = True
        # When you publish a message, the client returns a future.
        while(True):
            while self.ble.connected and any(
                UARTService in connection for connection in self.ble.connections
            ):
                if calibrate_still:
                    if(begin_calibrate):
                        print("Leave the white figurine neutral and do not touch it.")
                        input("Press enter to continue (do not touch figurine)")
                        print("beginning calibration....")
                        begin_calibrate = False

                    for connection in self.ble.connections:
                        if UARTService not in connection:
                            continue
                        uart = connection[UARTService]
                        one_byte = uart.read(20)
                        print("reading data point " + str(len(calib_mtx_mag)))
                        if one_byte:
                            #print(one_byte.decode('UTF-8'))
                            bs = one_byte.decode('UTF-8')
                            print(bs)
                            vals = bs.split('#')
                            xVal = int(vals[0]) - 100000
                            yVal = int(vals[1]) - 100000
                            bVal = int(vals[2]) - 100000

                            mag = self.get_magnitude(xVal, yVal)

                            calib_mtx_mag.append(mag)
                            calib_mtx_btn.append(bVal)
                            #time.sleep(1)
                        else:
                            print("couldn't read data point")
                        #time.sleep(1)

                        if len(calib_mtx_mag) >= 100:
                            calibrate_still = False
                            begin_calibrate = True
                            print("calculating offsets...")
                            offset_magnitude = sum(calib_mtx_mag) / len(calib_mtx_mag)
                            high_button = sum(calib_mtx_btn) / len(calib_mtx_btn)
                            print("finished calibration")
                            print("offset_magnitude value " + str(offset_magnitude))
                            print("button_high value " + str(high_button))
                elif calibrate_angle:
                    if(begin_calibrate):
                        print("Tip and hold the white figurine forward for calibration.")
                        input("Press enter to continue (and keep holding white figurine forward)")
                        print("beginning calibration....")
                        begin_calibrate = False

                    for connection in self.ble.connections:
                        if UARTService not in connection:
                            continue
                        uart = connection[UARTService]
                        one_byte = uart.read(20)
                        print("reading data point " + str(len(calib_mtx_angle)))
                        if one_byte:
                            #print(one_byte.decode('UTF-8'))
                            bs = one_byte.decode('UTF-8')
                            print(bs)
                            vals = bs.split('#')
                            xVal = int(vals[0]) - 100000
                            yVal = int(vals[1]) - 100000

                            direct = self.get_direction(xVal, yVal)

                            calib_mtx_angle.append(direct)
                            #time.sleep(1)
                        else:
                            print("couldn't read data point")
                        #time.sleep(1)

                        if len(calib_mtx_angle) >= 100:
                            calibrate_angle = False
                            print("calculating offsets...")
                            offset_angle = sum(calib_mtx_angle) / len(calib_mtx_angle)
                            print("finished calibration")
                            print("offset_angle value " + str(offset_angle))

                else:
                    for connection in self.ble.connections:
                        if UARTService not in connection:
                            continue
                        uart = connection[UARTService]
                        one_byte = uart.read(20)
                        print("reading data point")
                        if one_byte:
                            #print(one_byte.decode('UTF-8'))
                            bs = one_byte.decode('UTF-8')
                            print(bs)
                            vals = bs.split('#')
                            xVal = int(vals[0]) - 100000
                            yVal = int(vals[1]) - 100000
                            bVal = int(vals[2]) - 100000

                            #interp = classify(xVal, yVal, bVal)
                            mag = self.get_magnitude(xVal, yVal)
                            mag = mag - offset_magnitude
                            direct = self.get_direction(xVal, yVal)
                            direct = direct - offset_angle

                            if direct > 180:
                                print("correcting too high")
                                direct = direct - 360
                            elif direct < -180:
                                print("correcting too low")
                                direct = direct + 360

                            btn = math.floor(bVal * 10000 / high_button)

                            timestamp = time.time()
                            data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(bVal)
                            print(data)
                            
                            #print('sending ' + data)
                            data = bytes(data, 'UTF-8')
                            
                            #print(data)
                            print("-------")
                            try:
                                #print("sending to google cloud")
                                api_future = self.client.publish(self.topic_path, data)
                                api_future.add_done_callback(self.get_callback(api_future, data, ref))
                                #print("sent to google cloud")
                            except:
                                print("could not send to google cloud")

                            # Keep the main thread from exiting while the message future
                            # gets resolved in the background.
                            '''
                            while api_future.running():
                                time.sleep(0.5)
                                print("Published {} message(s).".format(ref["num_messages"]))
                            '''
                            #time.sleep(1)
                        else:
                            print("couldn't read data point")
                        #time.sleep(1)

            print("disconnected, scanning")
            #data=b"disconnected, scanning"
            #api_future = client.publish(topic_path, data)
            #api_future.add_done_callback(get_callback(api_future, data, ref))

            for advertisement in self.ble.start_scan(ProvideServicesAdvertisement, timeout=1):
                if UARTService not in advertisement.services:
                    continue
                self.ble.connect(advertisement)
                print("connected")
                break
            self.ble.stop_scan()

    '''
    def classify(x,y,button):
        if(button < 1):
            return "handoff"
        else:
            dir = get_direction(x,y)
            mag = get_magnitude(x,y)
            print(mag)
            if mag < 47 and mag > 45:
                return "no gesture detected"
            elif (mag > 60 or mag < 35):
                return "point@" + str(dir)
            else:
                return "gaze@" + str(dir)
    '''

    def get_direction(self, x, y):
        xlocal = (x/1000) - self.x0
        ylocal = (y/1000) - self.y0

        if xlocal == 0:
            if ylocal > 0:
                return(90)
            elif ylocal < 0:
                return (270)
            else:
                return ("NA")
        else:
            angle = math.atan(math.fabs(ylocal)/math.fabs(xlocal))
            if (ylocal < 0 and xlocal < 0):
                angle += math.pi
            elif (ylocal < 0 and xlocal > 0):
                angle = 2*math.pi - angle
            elif(ylocal > 0 and xlocal < 0):
                angle = math.pi - angle

            return (angle * 180 / 3.14)
        return 0

    def get_magnitude(self, x,y):
        z = math.sqrt(x**2 + y**2)
        return z

    #cleanup function
    def signal_handler(self, sig, frame):
        print('Deleting topic before closing...')
        self.client.delete_topic(self.topic_path)
        print('Finished cleanup, exiting!')
        sys.exit(0)

if __name__ == "__main__":
    gpn = Gesture_Publisher_Node('verification-of-hri', 'robot_topic', sys.argv[1])
    signal.signal(signal.SIGINT, gpn.signal_handler)
    if len(sys.argv) > 2 and sys.argv[2] == "test":
        gpn.test_pub()
    elif len(sys.argv) > 2 and sys.argv[2] == "test_wait":
        gpn.test_wait()
    elif len(sys.argv) > 2 and sys.argv[2] == "test_tilt":
        gpn.test_tilt()
    elif len(sys.argv) > 2 and sys.argv[2] == "test_point":
        gpn.test_point()
    else:
        gpn.pub()
# [END pubsub_quickstart_pub_all]