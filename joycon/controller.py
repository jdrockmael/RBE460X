#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16, Float32
from evdev import ecodes, InputDevice, ff, util
import time
import signal
import asyncio
leftEffectID=None
rightEffectID=None
leftLevel=0
rightLevel=0
rumbleUpdateTime=500 #ms
effectIdsLeft = [None] * 13
effectIdsRight = [None] * 13
stop = False
leftPublisher=None
rightPublisher=None
#start all subscribers and publishers while also starting the node
def initilize():
    global leftPublisher, rightPublisher
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joyconController', anonymous=True)

    rospy.Subscriber("leftRumbleLevel", Int16, leftlevelSetter)
    rospy.Subscriber("rightRumbleLevel", Int16, rightlevelSetter)
    

#if new level of left controller rumble is sent will update rumble for next loop
def leftlevelSetter(data):
    global leftLevel
    leftLevel=data.data

#if new level of right controller rumble is sent will update rumble for next loop
def rightlevelSetter(data):
    global rightLevel
    rightLevel=data.data


#grabs all the information related to the left controller
def getLeftControllerInfo():
    for name in util.list_devices():
        dev = InputDevice(name)
        if dev.name == 'Nintendo Switch Left Joy-Con':
            break
    if dev is None:
        raise Exception("No left controller found, confirm controller is connected") 
    return dev

#grabs all the information related to the right controller
def getRightControllerInfo():
    for name in util.list_devices():
        dev = InputDevice(name)
        if dev.name == 'Nintendo Switch Right Joy-Con':
            break
    if dev is None:
        raise Exception("No right controller found, confirm controller is connected") 
    return dev

#will cause rumble to be sent to controller and to start having it vibrate
def controllerRumble(controllerInfo,level):
    global leftEffectID, rightEffectID, effectIds
    empty=False
    repeat_count = 1
    if level > 12:
        level =12
    #check if the effect has ever been used before is so just use the old effect
    if controllerInfo.name=='Nintendo Switch Left Joy-Con' and effectIdsLeft[level] == None:
        print(effectIdsLeft)
        empty =True
    if controllerInfo.name=='Nintendo Switch Right Joy-Con' and effectIdsRight[level] == None:
        empty =True
    if empty == True:
        print('making new effect')
        if level ==0:
            rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0x0000)
        elif level ==1:
            rumble = ff.Rumble(strong_magnitude=0x1000, weak_magnitude=0x1000)
        elif level ==2:
            rumble = ff.Rumble(strong_magnitude=0x2000, weak_magnitude=0x2000)
        elif level ==3:
            rumble = ff.Rumble(strong_magnitude=0x3000, weak_magnitude=0x3000)
        elif level ==4:
            rumble = ff.Rumble(strong_magnitude=0x4000, weak_magnitude=0x4000)
        elif level ==5:
            rumble = ff.Rumble(strong_magnitude=0x5000, weak_magnitude=0x5000)
        elif level ==6:
            rumble = ff.Rumble(strong_magnitude=0x6000, weak_magnitude=0x6000)
        elif level ==7:
            rumble = ff.Rumble(strong_magnitude=0x7000, weak_magnitude=0x7000)
        elif level ==8:
            rumble = ff.Rumble(strong_magnitude=0x8000, weak_magnitude=0x8000)
        elif level ==9:
            rumble = ff.Rumble(strong_magnitude=0x9000, weak_magnitude=0x9000)
        elif level ==10:
            rumble = ff.Rumble(strong_magnitude=0xa000, weak_magnitude=0xa000)
        elif level ==11:
            rumble = ff.Rumble(strong_magnitude=0xb000, weak_magnitude=0xb000)
        else:
            rumble = ff.Rumble(strong_magnitude=0xc000, weak_magnitude=0xc000)
        effect_type = ff.EffectType(ff_rumble_effect=rumble)
        duration_ms = 1000

        effect = ff.Effect(
            ecodes.FF_RUMBLE, # type
            -1, # id (set by ioctl)
            0,  # direction
            ff.Trigger(0, 0), # no triggers
            ff.Replay(duration_ms, 0), # length and delay
            effect_type
        )
        
        print("Uploading FF effect...")
        effect_id = controllerInfo.upload_effect(effect)

        if controllerInfo.name=='Nintendo Switch Left Joy-Con':
            effectIdsLeft[level]=effect_id
        else:
            effectIdsRight[level]=effect_id
    else: 
        if controllerInfo.name=='Nintendo Switch Left Joy-Con':
            effect_id= effectIdsLeft[level]
            leftEffectID=effect_id
            
        else:
            effect_id= effectIdsRight[level]
            rightEffectID=effect_id
    controllerInfo.write(ecodes.EV_FF, effect_id, repeat_count)

#will take care of shutting down controllers if control c is pressed
def handler(signum, frame):
    global leftEffectID,leftControllerInfo, rightControllerInfo, rightEffectID, stop
    try:
        leftControllerInfo.erase_effect(leftEffectID) 
    except:
        print("left controller no event running")

    try:
        rightControllerInfo.erase_effect(rightEffectID)
    except:
        print("right controller no event running")
    stop=True  

#main function to run the rumble as it needs to constantly be updated
def mainCode():
    global rightLevel, leftLevel, stop
    signal.signal(signal.SIGINT, handler)

    print(util.list_devices())
    rightControllerInfo=getRightControllerInfo()
    rospy.loginfo('got left')
    leftControllerInfo=getLeftControllerInfo() 
    rospy.loginfo('got right')

    interval=rumbleUpdateTime/1000
    while not rospy.is_shutdown() and stop != True:
        rospy.loginfo('updating new rumble')
        controllerRumble(rightControllerInfo,rightLevel)
        controllerRumble(leftControllerInfo,leftLevel)
        time.sleep(interval)
    try:
        leftControllerInfo.erase_effect(leftEffectID) 
    except:
        print("left controller not event running")

    try:
        rightControllerInfo.erase_effect(rightEffectID)
    except:
        print("right controller not event running")
 
if __name__ == '__main__':
    initilize()
    mainCode()
