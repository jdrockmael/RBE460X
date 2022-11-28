#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import Twist
import math
import csv
import cv2 # OpenCV library
maxSpeed=.45 #m/s .9 of the robot max speed set in robotDrive.py
maxTurning=1 #rad/s
speed=0.0
turning=0
oldTime=0
mathInterval=15
firstVar=[]
secondVar=[]
thirdVar=[]
queastionIndex=0
def getCSV():
    global firstVar, secondVar, thirdVar
    rows = []
    # reading csv file
    nameFile= rospy.get_param("/csvFile")  
    with open(nameFile, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
    
        # extracting each data row one by one
        for row in csvreader:
            rows.append(row)
    for index in rows:
        data=index[0]
        first, second, third = data.split(":", 2)
        firstVar.append(first)
        if second == '0':
            secondVar.append('+')
        elif second == '1':
            secondVar.append('-')
        else:
            secondVar.append('x')
        thirdVar.append(third)
    print(firstVar)
def updateDrive(msg):
    global speed, turning
    speed=msg.linear.x
    turning=msg.angular.z
def callback(data):
    global speed, turning, maxSpeed, maxTurning, oldTime, mathInterval, firstVar, secondVar, thirdVar, queastionIndex
    global offsetCenter, widthSpeedBar, widthBar, radiusCircle, enableMath
    # Used to convert between ROS and OpenCV images
    br = CvBridge()


    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
   
    #Displays the speed
    (h, w) = current_frame.shape[:2]
    splitHeight=int(h/16)
    green=(0, 255, 0)
    red=(0,0,255)
    offsetCenter=50
    widthSpeedBar=10
    speedInicator=splitHeight+ int(splitHeight*(speed/maxSpeed))
    cv2.rectangle(current_frame, (int(w/2)-offsetCenter,h-splitHeight), (int(w/2)-offsetCenter-widthSpeedBar,h-splitHeight*2), green, 2)
    cv2.rectangle(current_frame, (int(w/2)-offsetCenter-1,h-splitHeight), (int(w/2)-offsetCenter-widthSpeedBar+1,h-speedInicator), red, -1)

    #diplays the turning circle
    radiusCircle=50
    percentTurn=turning/maxTurning
    angleTurn=percentTurn*(1.5708)+1.5708
    y=int(math.sin(angleTurn)*radiusCircle)
    x=int(math.cos(angleTurn)*radiusCircle)
    cv2.circle(current_frame,((int(w/2)+(offsetCenter*2),h-splitHeight-int(splitHeight/2))), radiusCircle, green, 1)
    cv2.line(current_frame, ((int(w/2)+(offsetCenter*2),h-splitHeight-int(splitHeight/2))), (int(w/2)+(offsetCenter*2)+x,h-splitHeight-int(splitHeight/2)-y), red, 2) 
    
    
    if enableMath == 1:
        # font
        font = cv2.FONT_HERSHEY_SIMPLEX
        # org
        org = (50, h-splitHeight)
        # fontScale
        fontScale = 3
        # Blue color in BGR
        color = (255, 0, 255)
        # Line thickness of 2 px
        thickness = 2
        # add queastion to display
        queastion=firstVar[queastionIndex]+ secondVar[queastionIndex] +thirdVar[queastionIndex]+'=?'
        # Using cv2.putText() method
        cv2.putText(current_frame, queastion, org, font, fontScale, color, thickness, cv2.LINE_AA)
        
        seconds = rospy.get_time()
        if seconds-oldTime > mathInterval or oldTime == 0:
            oldTime = seconds
            queastionIndex+=1


    # Display image
    #define the screen resulation
    screen_res = 1920, 1080
    scale_width = screen_res[0] / current_frame.shape[1]
    scale_height = screen_res[1] / current_frame.shape[0]
    scale = min(scale_width, scale_height)
    #resized window width and height
    window_width = int(current_frame.shape[1] * scale)
    window_height = int(current_frame.shape[0] * scale)
    #cv2.WINDOW_NORMAL makes the output window resizealbe
    cv2.namedWindow('Resized Window', cv2.WINDOW_NORMAL)
    #resize the window according to the screen resolution
    cv2.resizeWindow('Resized Window', window_width, window_height)
    cv2.imshow('Resized Window', current_frame)

    cv2.waitKey(1)
      
def receive_message():
    global mathInterval, offsetCenter, widthSpeedBar, widthBar, radiusCircle, maxSpeed, maxTurning, enableMath
    mathInterval= rospy.get_param("/mathInterval") 
    radiusCircle= rospy.get_param("/radiusCircle") 
    offsetCenter= rospy.get_param("/offsetCenter") 
    widthSpeedBar= rospy.get_param("/widthSpeedBar") 
    widthBar= rospy.get_param("/widthSensorBar")
    maxSpeed= rospy.get_param("/maxStraightSpeed") 
    maxSpeed=maxSpeed*.9
    maxTurning= rospy.get_param("/maxTurningSpeed") 
    enableMath= rospy.get_param("/enableMath") 
    getCSV()
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node('video_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    rospy.Subscriber('/cmd_vel', Twist, updateDrive)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()