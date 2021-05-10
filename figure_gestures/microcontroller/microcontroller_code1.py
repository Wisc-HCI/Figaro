# CircuitPython AnalogIn Demo
import time

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

import board
from analogio import AnalogIn

ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

analog_in_X = AnalogIn(board.A0)
analog_in_Y = AnalogIn(board.A1)
button = AnalogIn(board.A3)

def get_value(pin):
    return pin.value / 1000

while True:
    #print(get_voltage(analog_in_X), get_voltage(analog_in_Y), get_voltage(button))
    #print(classify(get_value(analog_in_X), get_value(analog_in_Y), get_value(button)))
    ble.start_advertising(advertisement)
    while not ble.connected:
        pass
    while ble.connected:
        # Returns b'' if nothing was read.
        one_byte = uart.read(4)
        if one_byte:
            one_byte = str(analog_in_X.value + 100000) + "#" + str(analog_in_Y.value + 100000) + "#" + str(button.value + 100000)
            print(one_byte)
            uart.write(one_byte)
    time.sleep(1)