#!/usr/bin/python3

"""Copyright (c) 2019, Douglas Otwell

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import traceback
import logging
import threading
import queue
import pigpio
import dbus
import os
import sys
import time
from optparse import OptionParser


from advertisement import Advertisement
from service import Application, Service, Characteristic, Descriptor
from drone import FlightController
from constants import Constants

GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
NOTIFY_TIMEOUT = 1000
RUNNING_MODE   = Constants.MODE_INDOOR

class DroneAdvertisement(Advertisement):
    def __init__(self, index):
        Advertisement.__init__(self, index, "peripheral")
        self.add_local_name("Drone")
        self.include_tx_power = True

class DroneService(Service):
    DRONE_SVC_UUID = "00000001-710e-4a5b-8d75-3e5b444bc3cf"

    def __init__(self, index):
        Service.__init__(self, index, self.DRONE_SVC_UUID, True)
        self.add_characteristic(DControlCharacteristic(self))
        self.add_characteristic(DStatusCharacteristic(self))
        self.fc = FlightController(RUNNING_MODE)
        self.messages = 0

    def quit(self):
        self.characteristics[0].taskQueue.put("cmd=quit")

class DStatusCharacteristic(Characteristic):
    DSTATUS_CHARACTERISTIC_UUID = "00000002-710e-4a5b-8d75-3e5b444bc3cf"

    def __init__(self, service):
        self.notifying = False
        Characteristic.__init__(self, self.DSTATUS_CHARACTERISTIC_UUID,["notify", "read"], service)
        self.add_descriptor(DStatusDescriptor(self))

    def get_drone_status(self):
        value = []
        for c in self.service.fc.get_status():
            value.append(dbus.Byte(c.encode()))
        return value

    def update_drone_status_callback(self):
        if self.notifying:
            value = self.get_drone_status()
            self.messages += 1
            self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": value}, [])
        return self.notifying

    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        self.messages = 0
        self.add_timeout(NOTIFY_TIMEOUT, self.update_drone_status_callback)

    def StopNotify(self):
        self.notifying = False

    def ReadValue(self, options):
        value = self.get_drone_status()
        return value

class DStatusDescriptor(Descriptor):
    DSTATUS_DESCRIPTOR_UUID = "2901"
    DSTATUS_DESCRIPTOR_VALUE = "Dron status"

    def __init__(self, characteristic):
        Descriptor.__init__(self, self.DSTATUS_DESCRIPTOR_UUID, ["read"], characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.DSTATUS_DESCRIPTOR_VALUE
        for c in desc:
            value.append(dbus.Byte(c.encode()))
        return value

class DControlCharacteristic(Characteristic):
    DCONTROL_CHARACTERISTIC_UUID = "00000003-710e-4a5b-8d75-3e5b444bc3cf"

    def __init__(self, service):
        Characteristic.__init__(
                self, self.DCONTROL_CHARACTERISTIC_UUID,
                ["read", "write"], service)
        self.add_descriptor(DcontrolDescriptor(self))
        self.taskQueue = queue.Queue()
        threading.Thread(target = self.executeCommand).start()

    def WriteValue(self, value, options):
        val = ""
        for c in value:
            val = val + str(c)
        #logging.info("[BT]Received: " + val)
        if val == "cmd=shutdown":
            os.system("sudo pkill pigpiod")
            os.system("sudo shutdown now")
        elif val == "cmd=ping":
            self.service.fc.ping()
        else:
            self.taskQueue.put(val)

    def ReadValue(self, options):
        value = []
        value.append(dbus.Byte(self.service.fc.get_status().encode()))
        return value

    def executeCommand(self):
        done = False
        while not done:
            cmd = self.taskQueue.get()
            self.taskQueue.task_done()
            #logging.info("[RX]" + cmd)
            components = cmd.split(";")
            for component in components:
                name_value = component.split("=")
                name = name_value[0]
                value= name_value[1]
                if "quit" == value:
                    done = True
                try:
                    self.doCommand(name, value)
                except Exception as e:
                    logging.error(traceback.format_exc())

    def doCommand(self, name, value):
        if name == "cmd":
            if "calibrate" == value:
                self.service.fc.calibrate()
            elif "cali_imu" == value:
                self.service.fc.calibrate_imu()
            elif "cali_mag" == value:
                self.service.fc.calibrate_compass()
            elif "start" == value:
                self.service.fc.start()
            elif "takeoff" == value:
                self.service.fc.takeoff()
            elif "landing" == value:
                self.service.fc.landing()
            elif "turn_left" == value:
                self.service.fc.turn_left()
            elif "turn_right" == value:
                self.service.fc.turn_right()
            elif "move_forward" == value:
                self.service.fc.move_forward()
            elif "move_backward" == value:
                self.service.fc.move_backward()
            elif "move_left" == value:
                self.service.fc.move_left()
            elif "move_right" == value:
                self.service.fc.move_right()
            elif "hover" == value:
                self.service.fc.hover()
            elif "home" == value:
                self.service.fc.home()
            elif "+" == value:
                self.service.fc.inc_speed()
            elif "-" == value:
                self.service.fc.dec_speed()
            elif "stop" == value:
                self.service.fc.stop()
            elif "ping" == value: #will not go here
                self.service.fc.ping()
            elif "reset" == value:
                self.service.fc.reset()
            elif "quit" == value:
                self.service.fc.shutdown()
        elif name == "config":
            self.service.fc.set_config(value)

class DcontrolDescriptor(Descriptor):
    DCONTROL_DESCRIPTOR_UUID = "2901"
    DCONTROL_DESCRIPTOR_VALUE = "Drone Flight Controller"

    def __init__(self, characteristic):
        Descriptor.__init__(self, self.DCONTROL_DESCRIPTOR_UUID, ["read"], characteristic)

    def ReadValue(self, options):
        value = []
        desc = self.DCONTROL_DESCRIPTOR_VALUE
        for c in desc:
            value.append(dbus.Byte(c.encode()))
        return value

def handle_unhandled_exception(exc_type, exc_value, exc_traceback):
    logging.error("An unnhandled exception occurred. The system will shutdown soon")
    logging.error(traceback.format_exception(exc_type, exc_value, exc_traceback))
    os.system("sudo pkill pigpiod")
    os.system("sudo shutdown now")
    
parser = OptionParser()
parser.add_option("-m", "--mode", dest="mode", help="Running mode.")
(options, args) = parser.parse_args()

#if serial no not specified
if options.mode == "outdoor": #command line: python3 main.py -m outdoor > /dev/null &
    RUNNING_MODE = Constants.MODE_OUTDOOR #"sudo kill -SIGINT process_id" to terminate it
elif options.mode == "training":
    RUNNING_MODE = Constants.MODE_TRAINING    
else:
    RUNNING_MODE = Constants.MODE_INDOOR

if os.path.exists("drone_old.log"):
    os.remove("drone_old.log")
if os.path.exists("drone.log"):
    os.rename("drone.log", "drone_old.log")

logging.basicConfig(filename="drone.log",
                    filemode='a',
                    format='%(asctime)s.%(msecs)03d %(levelname)s %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.DEBUG)

if RUNNING_MODE == Constants.MODE_INDOOR or RUNNING_MODE == Constants.MODE_TRAINING:
    # define a Handler which writes DEBUG messages or higher to the sys.stderr
    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG)
    console.setFormatter(logging.Formatter('%(levelname)-8s %(message)s'))
    # add the handler to the root logger
    logging.getLogger().addHandler(console)

if RUNNING_MODE == Constants.MODE_OUTDOOR:
    sys.excepthook = handle_unhandled_exception

app = Application()
app.add_service(DroneService(0))
app.register()

adv = DroneAdvertisement(0)
adv.register()

try:
    app.run()
except KeyboardInterrupt:
    app.services[0].quit() #drone service
    time.sleep(2)
    app.quit()
