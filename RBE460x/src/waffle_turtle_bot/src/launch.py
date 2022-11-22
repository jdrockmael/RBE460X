#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

tau = 2 * math.pi
class Lab2:
    
    def __init__(self):
        """
        Class constructor
        """
        self.px, self.py, self.pth = 0, 0, 0
        
        rospy.init_node("lab2")
        self.pubCMDVEL = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        
        rospy.Subscriber("odom", Odometry, self.update_odometry)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.go_to)

    # publisher
    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # Twist, linear
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed # X is forward
        msg_cmd_vel.linear.y = 0
        msg_cmd_vel.linear.z = 0
        # angular
        msg_cmd_vel.angular.x = 0
        msg_cmd_vel.angular.y = 0
        msg_cmd_vel.angular.z = angular_speed # Yaw is Z
        
        self.pubCMDVEL.publish(msg_cmd_vel)
        
    # Functions to drive the robot.
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        print("Driving %s m at %s m/s" %(distance, linear_speed))
        
        tolerance = 0.01 # m
        startX = self.px # Get start position
        startY = self.py
        
        traveled = 0 # distance traveled
        while traveled < distance - tolerance: # While it needs to drive,
            print("Distance remaining: %s m" % (distance - traveled))
            self.send_speed(linear_speed, 0) # make it go
            traveled = math.sqrt(pow(self.px - startX, 2) + pow(self.py - startY, 2)) # get distance from start
            rospy.sleep(0.01) # refresh rate
        self.send_speed(0, 0) # stop

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        threshold = 0.05 # rad
        currAngle = self.pth # Starting angle
        startingAngle = currAngle # this one's static
        
        angleremaining = abs(angle) # amount remaining to turn
        targetAngle = (currAngle + angle) % tau # get final angle, normalizing it.

        if angle < 0: # Get direction for motor commands
            aspeed *= -1

        while angleremaining > threshold: # while it needs to turn
            self.send_speed(0, math.copysign(aspeed, angle)) # turn
            currAngle = self.pth # update angle
            angleremaining = abs(targetAngle - currAngle) % tau # amount remaining, normalized
            
            rospy.sleep(0.01) # refresh rate
        
        self.send_speed(0, 0) # stop
        rospy.sleep(0.5) # wait a bit for it to settle (helps when driving straight after turning)

    # subscribers
    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # Position
        finalPose = msg.pose
        finalPoint = finalPose.position
        finalX = finalPoint.x
        finalY = finalPoint.y

        # Orientation
        finalQuat = finalPose.orientation
        quat_list = [finalQuat.x, finalQuat.y, finalQuat.z, finalQuat.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        finalYaw = yaw

        poseHeading = math.atan2(finalY - self.py, finalX - self.px) # Heading from current position to target
        turnToFinalPoint = (poseHeading - self.pth + math.pi) % tau - math.pi # Angle needed to turn to that heading
        
        self.rotate(turnToFinalPoint, 0.5)

        # Get drive distance. This is done after turning to account for any drift in turning
        dist = math.sqrt(math.pow(finalX - self.px, 2) + math.pow(finalY - self.py, 2))
        
        self.drive(dist, 0.15)

        angle = (finalYaw - self.pth + math.pi) % tau - math.pi
        self.rotate(angle, 0.5)


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation # gets the quaternions
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list) # converts them into RPY angles
        self.pth = yaw # only care about the yaw


    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code

    def run(self):
        rospy.sleep(4)
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()
