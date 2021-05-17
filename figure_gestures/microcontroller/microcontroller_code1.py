# Code for the figurine microcontroller. Constantly transmits the X and Y of the joystick orientation, as well as the button voltage.
import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

import board
from analogio import AnalogIn

#start the BLE service
ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

#set the physical connections
analog_in_X = AnalogIn(board.A0)
analog_in_Y = AnalogIn(board.A1)
button = AnalogIn(board.A3)

#pin values are very large, so we scale it to be smaller with this function
def get_value(pin):
    return pin.value / 1000

#continuously streams the X, Y, and button voltages
while True:
    #print(get_voltage(analog_in_X), get_voltage(analog_in_Y), get_voltage(button))
    #print(classify(get_value(analog_in_X), get_value(analog_in_Y), get_value(button)))
    
    ble.start_advertising(advertisement)
    #if there's no connection, do nothing
    while not ble.connected:
        pass
    #if there is a device connected, do something
    while ble.connected:
        # Returns b'' if nothing was read.
        #read the byte values
        one_byte = uart.read(4)
        if one_byte:
            #build a string of the X, Y, and button voltages
            #since the length of the string needs to be consistent and is up to 5 digits, 
            #we pad it with 1e5,then process away the irrelevant numbers at the other endof communication
            one_byte = str(analog_in_X.value + 100000) + "#" + str(analog_in_Y.value + 100000) + "#" + str(button.value + 100000)
            print(one_byte)
            uart.write(one_byte)
    time.sleep(1)
