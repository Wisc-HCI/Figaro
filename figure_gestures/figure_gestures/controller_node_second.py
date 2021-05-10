import time
import math
from datetime import datetime
import signal
import sys
import json

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

    def get_callback(self, api_future, data, ref):
        """Wrap message data in the context of the callback function."""

        def callback(api_future):
            try:
                print(
                    "Published message {} now has message ID {}".format(
                        data, api_future.result()
                    )
                )
                ref["num_messages"] += 1
            except Exception:
                print(
                    "A problem occurred when publishing {}: {}\n".format(
                        data, api_future.exception()
                    )
                )
                raise

        return callback


    def pub(self):
        # Keep track of the number of published messages.
        ref = dict({"num_messages": 0})

        # When you publish a message, the client returns a future.
        while(True):
            while self.ble.connected and any(
                UARTService in connection for connection in self.ble.connections
            ):
                for connection in self.ble.connections:
                    if UARTService not in connection:
                        continue
                    #print("echo")
                    test = "echo"
                    uart = connection[UARTService]
                    uart.write(test.encode('UTF-8'))
                    # Returns b'' if nothing was read.
                    one_byte = uart.read(20)
                    if one_byte:
                        #print(one_byte.decode('UTF-8'))
                        bs = one_byte.decode('UTF-8')
                        print(bs)
                        vals = bs.split('.')
                        xVal = int(vals[0]) - 100000
                        yVal = int(vals[1]) - 100000
                        bVal = int(vals[2]) - 100000

                        #interp = classify(xVal, yVal, bVal)
                        mag = self.get_magnitude(xVal, yVal)
                        direct = self.get_direction(xVal, yVal)
                        timestamp = datetime.utcnow()
                        data = str(timestamp) + "#" + self.actor + "#" + str(mag) + "#" + str(direct) + "#" + str(bVal)

                        print('sending ' + data)
                        data = bytes(data, 'UTF-8')
                        print(data)
                        api_future = self.client.publish(self.topic_path, data)
                        api_future.add_done_callback(self.get_callback(api_future, data, ref))

                        # Keep the main thread from exiting while the message future
                        # gets resolved in the background.
                        '''
                        while api_future.running():
                            time.sleep(0.5)
                            print("Published {} message(s).".format(ref["num_messages"]))
                        '''
                        time.sleep(1)

                    #time.sleep(.001)

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
        xlocal = x - self.x0
        ylocal = y - self.y0

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
    gpn.pub()
# [END pubsub_quickstart_pub_all]