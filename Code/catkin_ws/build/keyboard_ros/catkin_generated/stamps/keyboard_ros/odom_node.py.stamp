# !/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
from math import cos, sin
import tf

class WheelEncoderOdom:
    def __init__(self):
        rospy.init_node('wheel_encoder_odom')
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        
        self.wheelbase = 0.24  # Distance between wheels (adjust as needed)
        self.wheel_radius = 0.0325  # Wheel radius (adjust as needed)
        self.ticks_per_revolution = 515  # Ticks per revolution (adjust as needed)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber('/wheel_left_ticks', Int16, self.left_ticks_callback)
        rospy.Subscriber('/wheel_right_ticks', Int16, self.right_ticks_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
    def left_ticks_callback(self, msg):
        self.left_ticks = msg.data
        
    def right_ticks_callback(self, msg):
        self.right_ticks = msg.data
        
    def calculate_odometry(self):
        delta_left_ticks = self.left_ticks - self.left_ticks_prev
        delta_right_ticks = self.right_ticks - self.right_ticks_prev
        
        distance_left = (delta_left_ticks * 2 * 3.14159 * self.wheel_radius) / self.ticks_per_revolution
        distance_right = (delta_right_ticks * 2 * 3.14159 * self.wheel_radius) / self.ticks_per_revolution
        
        delta_distance = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.wheelbase
        
        self.pose_x += delta_distance * cos(self.pose_theta)
        self.pose_y += delta_distance * sin(self.pose_theta)
        self.pose_theta += delta_theta
        
        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks
         
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        odom_msg.pose.pose.position.z = 0.0
        
        quat = quaternion_from_euler(0, 0, self.pose_theta)
        odom_msg.pose.pose.orientation = Quaternion(*quat)
        
        odom_msg.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        
        self.odom_pub.publish(odom_msg)
        
        ##################################################
        ### broadcast the transformations
        ##################################################
        # since all odometry is 6DOF we'll need a quaternion created from yaw

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose_theta)
        current_time = rospy.Time.now()
        odom_broadcaster = tf.TransformBroadcaster()
        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
        (self.pose_x, self.pose_y , 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom" )
        
        ##################################################
        # odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose_theta)
        # current_time = rospy.Time.now()
        # odom_broadcaster = tf.TransformBroadcaster()
        # # first, we'll publish the transform over tf
        # odom_broadcaster.sendTransform(
        # (self.pose_x, self.pose_y , 0.),
        # odom_quat,
        # current_time,
        # "odom",
        # "base_link" )


    def run(self):
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.publish_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        wheel_encoder_odom = WheelEncoderOdom()
        wheel_encoder_odom.run()
    except rospy.ROSInterruptException:
        pass



# import math
# from math import sin, cos, pi

# import rospy
# import tf
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Int16
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# #Parameters
# wheeltrack = 0.24
# wheelradius = 0.0325
# TPR = 515
# left_ticks = 0
# right_ticks = 0
# last_left_ticks = 0
# last_right_ticks = 0

# x = 0.0
# y = 0.0
# th = 0.0

# vx =  0.0
# vy =  0.0
# vth =  0.0

# def leftTicksCallback(msg):
#     global left_ticks 
#     left_ticks = msg.data

# def rightTicksCallback(msg):
#     global right_ticks 
#     right_ticks = msg.data

# rospy.init_node('odometry_publisher')

# odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
# left_ticks_sub =rospy.Subscriber("/wheel_left_ticks", Int16, leftTicksCallback)
# right_ticks_sub =rospy.Subscriber("/wheel_right_ticks", Int16, rightTicksCallback)
# odom_broadcaster = tf.TransformBroadcaster()

# current_time = rospy.Time.now()
# last_time = rospy.Time.now()

# r = rospy.Rate(10)

# while not rospy.is_shutdown():
#     current_time = rospy.Time.now()

#     delta_L = left_ticks - last_left_ticks
#     delta_R = right_ticks - last_right_ticks
#     dl = 2 * pi * wheelradius * delta_L / TPR
#     dr = 2 * pi * wheelradius * delta_R / TPR
#     dc = (dl + dr) / 2
#     dt = (current_time - last_time).to_sec()
#     dth = (dr-dl)/wheeltrack

#     if dr==dl:
#         dx=dr*cos(th)
#         dy=dr*sin(th)

#     else:
#         radius=dc/dth

#         iccX=x-radius*sin(th)
#         iccY=y+radius*cos(th)

#         dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
#         dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

#     x += dx  
#     y += dy 
#     th =(th+dth) %  (2 * pi)

#     odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

#     # first, we'll publish the transform over tf
#     odom_broadcaster.sendTransform(
#        (x, y, 0.),
#        odom_quat,
#        current_time,
#        "base_link",
#        "odom"
#     )

#     # next, we'll publish the odometry message over ROS
#     odom = Odometry()
#     odom.header.stamp = current_time
#     odom.header.frame_id = "odom"

#     odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

#     if dt>0:
#        vx=dx/dt
#        vy=dy/dt
#        vth=dth/dt

#     odom.child_frame_id = "base_link"
#     odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

#     odom_pub.publish(odom)

#     last_left_ticks = left_ticks
#     last_left_ticks = right_ticks
#     last_time = current_time
#     r.sleep()