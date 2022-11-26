#!/usr/bin/env python
import rospy
import asyncio, evdev
from evdev import ecodes, InputDevice, ff, util
from std_msgs.msg import Float32
from select import select
loop=None
stop = False
def initilize():
    global leftPublisher, rightPublisher
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joyconJoyStickPub', anonymous=True)

    leftPublisher = rospy.Publisher('leftStickPos', Float32, queue_size=10)
    rightPublisher = rospy.Publisher('rightStickPos', Float32, queue_size=10)

#grabs all the information related to the left controller
def getLeftControllerInfo():
    dev=None
    for name in util.list_devices():
        dev = InputDevice(name)
        if dev.name == 'Nintendo Switch Left Joy-Con':
            break
    if dev is None:
        raise Exception("No left controller found, confirm controller is connected") 
    return name

#grabs all the information related to the right controller
def getRightControllerInfo():
    dev=None
    for name in util.list_devices():
        dev = InputDevice(name)
        if dev.name == 'Nintendo Switch Right Joy-Con':
            break
    if dev is None:
        raise Exception("No right controller found, confirm controller is connected") 
    return name

#will take care of shutting down controllers if control c is pressed
def handler(signum, frame):
    global stop
    rospy.loginfo('shutting down')
    stop = True

def mainCode():
    global leftPublisher, rightPublisher, stop
    rate = rospy.Rate(30) # 10hz
    leftDevice=getLeftControllerInfo()
    rightDevice=getRightControllerInfo()
    devices = map(InputDevice, (leftDevice, rightDevice))
    devices = {dev.fd: dev for dev in devices}

    while not rospy.is_shutdown() and stop != True:
        rate.sleep()
        r, w, x = select(devices, [], [],.1)
        for fd in r:
            for event in devices[fd].read():
                data=event.value/32300
                if event.code == 1:
                    leftPublisher.publish(data)
                if event.code == 4:
                    rightPublisher.publish(data)


if __name__ == '__main__':
    initilize()
    mainCode()