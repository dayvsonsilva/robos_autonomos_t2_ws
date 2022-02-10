#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath

import plotly.express as px
import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc
import dash_html_components as html
import plotly
import random
import plotly.graph_objs as go
from collections import deque


class Explorer:
    def __init__(self):
        self.points = np.zeros([1, 2])
        self.grid = np.zeros([10000, 10000])
        self.point = [0, 0]
        self.origin = [len(self.grid[0]) / 2, len(self.grid[1]) / 2]
        # self.origin = [0, 0]
        rospy.loginfo("Origen do mapa:" + str(self.origin))

        self.publisher_interval = 1.0

        self.vel_pub = rospy.Publisher(
            "/jackal_velocity_controller/cmd_vel", Twist, queue_size=10
        )
        self.timer_pub = rospy.Timer(
            rospy.Duration(self.publisher_interval), self.timerCallback
        )
        self.laser_sub = rospy.Subscriber(
            "/front/scan", LaserScan, self.laserCallback, queue_size=10
        )
        self.odometry_sub = rospy.Subscriber(
            "/jackal_velocity_controller/odom", Odometry, self.odometryCallback, queue_size=10)

    def timerCallback(self, event):
        rospy.loginfo("Timer callback")
        # msg = Twist()
        # msg.angular.z = 0.1
        # msg.linear.x = 0.2
        # self.vel_pub.publish(msg)
        # plt.scatter(self.points[:, 0], self.points[:, 1], c="r")
        # plt.show()
        # fig = px.scatter(x=self.points[:, 0], y=self.points[:, 1])
        # fig.show()

    def laserCallback(self, msg):
        rospy.loginfo("Laser callback")
        # rospy.loginfo("Resposta laser: " + str(msg.angle_increment))
        self.laser = msg.ranges
        self.angle_step = math.degrees(msg.angle_increment) / 57.2958
        self.angle_start = math.degrees(msg.angle_min)
        self.angle_stop = math.degrees(msg.angle_max)
        # rospy.loginfo("Laser len:" + str(len(self.laser)))
        self.points = np.zeros([1, 2])
        angulo = self.angle_start

        for i in range(len(self.laser)):
            self.point = np.array(
                [[(cmath.rect(self.laser[i], angulo).real), (cmath.rect(self.laser[i], angulo).imag)]])  # x e y
            angulo = angulo + self.angle_step  # self.angle_step
            self.points = np.concatenate([self.point, self.points])
        # rospy.loginfo("pontos:" + str(self.points))
        plt.scatter(self.points[:, 0], self.points[:, 1], c="r")
        plt.show(block=True)
        # fig = px.scatter(x=self.points[:, 0], y=self.points[:, 1])
        # fig.show()

    def odometryCallback(self, msg):
        rospy.loginfo("Laser callback")
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.orientation_w = msg.pose.pose.orientation.w
        rospy.loginfo("pose x: " + str(self.pose_x) + "y: " +
                      str(self.pose_x) + "w: " + str(self.orientation_w))

    def calc_area(self, msg):

        self.laser = msg.intensities
        self.angle = msg.angle_increment
        print(type(self.laser))


if __name__ == "__main__":
    try:
        rospy.init_node("explorer")
        Explorer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
