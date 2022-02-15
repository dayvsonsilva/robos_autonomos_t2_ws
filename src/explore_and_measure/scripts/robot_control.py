#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import PyKDL
from math import radians, pi, degrees
import numpy as np
import matplotlib.pyplot as plt
import time
import cmath


class RobotControl:
    def __init__(self):
        rospy.init_node("explorer")
        # Definição de mensagens recebidas e enviadas
        self.cmd = Twist()
        self.odometry_msg = Odometry()
        self.laser_msg = LaserScan()
        self.odometry_fil_msg = Odometry()
        # variaveis
        self.angular_tolerance = radians(2)
        self.rate = rospy.Rate(10)
        self.ctrl_c = False
        self.publisher_interval = 1.0

        
        self.tf_listener = tf.TransformListener()
        # Pub and sub nodes
        self.vel_pub = rospy.Publisher(
            "/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber(
            "/jackal_velocity_controller/odom", Odometry, self.odometryCallback, queue_size=10)
        self.laser_sub = rospy.Subscriber(
            "/front/scan", LaserScan, self.laserCallback, queue_size=10)
        self.odometry_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odometryFilCallback, queue_size=10)

    def laserCallback(self, msg):
        # rospy.loginfo("Laser callback")
        self.laser_msg = msg

    def odometryCallback(self, msg):
        # rospy.loginfo("Odometry callback")
        self.odometry_msg = msg

    def odometryFilCallback(self, msg):
        # rospy.loginfo("Odometry fill callback")
        self.odometry_fil_msg = msg

    def get_laser(self):
        time.sleep(1)
        msg = self.laser_msg
        return msg

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.vel_pub.get_num_connections()
            # summit_connections = self.summit_vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(self.cmd)
                # self.summit_vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    # retorna x(m), y(m) e w(degress)
    def get_pose(self):
        msg = self.odometry_msg
        msg2 = self.odometry_fil_msg
        self.pose_x = msg2.pose.pose.position.x
        self.pose_y = msg2.pose.pose.position.y

        orient_x = msg2.pose.pose.orientation.x
        orient_y = msg2.pose.pose.orientation.y
        orient_z = msg2.pose.pose.orientation.z
        orient_w = msg2.pose.pose.orientation.w

        self.orientation_w = self.quat_to_angle(
            Quaternion(orient_x, orient_y, orient_z, orient_w))

        self.orientation_w = degrees(self.orientation_w)
        # rospy.loginfo("orientation" + str(self.orientation_w))

        # x(m), y(m), w(degrees)
        return self.pose_x, self.pose_y, self.orientation_w

    def stop_robot(self):
        #rospy.loginfo("Stop robô.")
        # msg = Twist()
        # msg.angular.z = 0.0
        # msg.linear.x = 0.0
        #rospy.loginfo("Stop robô.")
        # self.vel_pub.publish(msg)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def linar_moviment(self, vel):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = vel
        self.vel_pub.publish(msg)
        self.publish_once_in_cmd_vel()

    def move_straight(self, vel):
        self.cmd.linear.x = vel
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()

    def angular_moviment(self, vel):
        msg = Twist()
        msg.angular.z = vel
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)

    def rotate(self, degrees):  # remover
        position = Point()
        (position, rotation) = self.get_odom()
        # Set the movement command to a rotation
        if degrees > 0:
            self.cmd.angular.z = 0.3
        else:
            self.cmd.angular.z = -0.3
        # Track the last angle measured
        last_angle = rotation
        # Track how far we have turned
        turn_angle = 0
        goal_angle = radians(degrees)
        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.vel_pub.publish(self.cmd)
            self.rate.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()

            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)

            turn_angle += delta_angle
            last_angle = rotation

        self.stop_robot()

    def get_odom(self):
        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(
                    '/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                'odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == "__main__":
    try:
        rospy.init_node("explorer")
        RobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
