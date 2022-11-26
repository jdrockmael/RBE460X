import rospy
from std_msgs.msg import Float32
from evdev import ecodes, InputDevice, ff, util
import time
import signal
import asyncio
from geometry_msgs.msg import Twist
leftStickPosition=0.0 #start the robot not moving
rightStickPosition=0.0 #start the robot not moving
maxStraightSpeed = 0.5 #m/s
maxTurningSpeed=1.0 #rad/s
toleranceMatchGoStraight=0.1 #how much the sticks must match to count as going straight

def initilize():
    global drivePublisher, rate
    rospy.init_node('driveRobot', anonymous=True)
    rospy.Subscriber("leftStickPos", Float32, updateLeftPos)
    rospy.Subscriber("rightStickPos", Float32, updateRightPos)
    drivePublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

def updateLeftPos(data):
    global leftStickPosition
    position=data.data
    if position > 0.9:
        position = 0.9
    if position < -0.9:
        position = -0.9
    leftStickPosition= position

def updateRightPos(data):
    global rightStickPosition
    position=data.data
    if position > 0.9:
        position = 0.9
    if position < -0.9:
        position = -0.9
    rightStickPosition= position

def turning(leftPos,rightPos):
    if rightPos < 0:
        valueright=1-abs(rightPos)
    else:
        valueright=abs(rightPos)+0.9
    if leftPos < 0:
        valueleft=0.9-abs(leftPos)
    else:
        valueleft=abs(leftPos)+1
    average=(leftPos+rightPos)/2
    if average >0:
        differance=valueright-valueleft
    else:
        differance=valueleft-valueright
    turningSpeed=(differance/1.8)*maxTurningSpeed
    if average > 0:
        turningSpeed=turningSpeed*-1
    
    return turningSpeed,average*-1

def updateBot(forward, turning):
    global drivePublisher
    newCommand=Twist()
    newCommand.linear.x=forward
    newCommand.linear.y=0.0
    newCommand.linear.z=0.0
    #turning
    newCommand.angular.x=0.0
    newCommand.angular.y=0.0
    newCommand.angular.z=turning
    drivePublisher.publish(newCommand)

def mainCode():
    global rightStickPosition, leftStickPosition, maxStraightSpeed, toleranceMatchGoStraight, maxTurningSpeed, rate
    #check if the stick are in roughly the same spot
    while not rospy.is_shutdown():
        if (rightStickPosition+toleranceMatchGoStraight)>leftStickPosition and (rightStickPosition-toleranceMatchGoStraight)<leftStickPosition:
            straightSpeed = leftStickPosition*maxStraightSpeed*-1
            if leftStickPosition >= 0:
                straightSpeed=straightSpeed
            turningSpeed=0.0
            #rospy.loginfo('straight')
        else:
            turningSpeed,percentSpeed=turning(leftStickPosition,rightStickPosition)
            straightSpeed=percentSpeed*maxStraightSpeed
            #rospy.loginfo('turning')
        # rospy.loginfo('straight')    
        # rospy.loginfo(straightSpeed)
        # rospy.loginfo('turning')
        # rospy.loginfo(turningSpeed)
        updateBot(straightSpeed,turningSpeed)
        rate.sleep()

        
if __name__ == '__main__':
    initilize()
    mainCode()