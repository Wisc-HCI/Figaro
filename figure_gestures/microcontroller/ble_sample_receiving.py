"""
Reads the value transmitted by the microcontroller, and decodes them.
Runs without ROS, for debug purposes.
"""

import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

# setup ble connection
ble = BLERadio()


# read the value and print
while True:
    while ble.connected and any(
        UARTService in connection for connection in ble.connections
    ):
        for connection in ble.connections:
            if UARTService not in connection:
                continue
            #print("echo")
            test = "echo"
            uart = connection[UARTService]
            uart.write(test.encode('UTF-8'))
            # Returns b'' if nothing was read.
            one_byte = uart.read(13)
            if one_byte:
                #print(one_byte.decode('UTF-8'))
                vals = one_byte.decode('UTF-8').split('.')
                xVal = int(vals[0]) - 100000
                yVal = int(vals[1]) - 100000
                print('sending ' + str(xVal) + ' and ' + str(yVal))
        time.sleep(.001)

    print("disconnected, scanning")
    for advertisement in ble.start_scan(ProvideServicesAdvertisement, timeout=1):
        if UARTService not in advertisement.services:
            continue
        ble.connect(advertisement)
        print("connected")
        break
    ble.stop_scan()
