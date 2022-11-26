#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int16, Float32
#rows are the individual points colums are the differnt zones for differnt vibrations 0 no vibration -> 15 max vibration
minDistanseFromBot=165 #mm based on roughly the middle of the lidar
widthOfScan=160 #mm
maxDistanceFromBot= 1000 #mm
colums = 13 #the number of differnt vibration settings
def publishData(left,right):
    global leftPublisher, rightPublisher
    data= left
    leftPublisher.publish(data)
    data=right
    rightPublisher.publish(data)
def callback(msg):
    global minAngle, listMaxLengths, leftZoneInfo, rightZoneInfo,colums
    leftZoneInfo=0
    rightZoneInfo=0

    for index in range(minAngle):
        sensorReading=msg.ranges[leftZoneInfo]
        #check if over the range of the max zone otherwise must be with in a zone
        if sensorReading > listMaxLengths[index][0]:
            pass
        else:
            for position in range(colums-1):
                if sensorReading > listMaxLengths[index][position+1] and sensorReading < listMaxLengths[index][position]:
                    leftZoneInfo=position+1
                    break

        #right side of the robot
        newSensorIndex=359-index
        sensorReading=msg.ranges[newSensorIndex]
        #left side of the robot
        if sensorReading > listMaxLengths[index][0]:
            pass
        else:
            #check if over the range of the max zone otherwise must be with in a zone
            for position in range(0,colums-1):
                if sensorReading > listMaxLengths[index][position+1] and sensorReading < listMaxLengths[index][position]:
                    rightZoneInfo=position+1
                    break
    publishData(leftZoneInfo,rightZoneInfo)

minAngle= math.degrees(math.atan(minDistanseFromBot/widthOfScan))
minAngle=int(minAngle)
listMaxLengths = [[0 for i in range(colums)] for j in range(minAngle)]
zoneSize=maxDistanceFromBot/colums
zoneDistance=maxDistanceFromBot
for distance in range(colums):
    for angle in range(0,minAngle):
        radAngle=math.radians(angle)
        if radAngle == 0:
            hypot = zoneDistance
        else:
            hypot=zoneDistance/math.cos(radAngle)
            adj=zoneDistance*math.tan(radAngle)
            if adj > widthOfScan:
                hypot=widthOfScan/math.sin(radAngle)
        listMaxLengths[angle][distance]=hypot/1000
    zoneDistance=zoneDistance-zoneSize
#print(listMaxLengths)
rospy.init_node('scan_values')
leftPublisher = rospy.Publisher('leftRumbleLevel', Int16, queue_size=10)
rightPublisher = rospy.Publisher('rightRumbleLevel', Int16, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()