#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath


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

    def pol_cart(self, raio, theta):
        # theta = math.degrees(theta)
        # theta = theta * (180 / math.pi)
        x = raio * np.cos(theta)  # * 1000
        y = raio * np.sin(theta)  # * 1000
        # x, y = cmath.polar(raio, theta)
        return (round(x), round(y))

    def timerCallback(self, event):
        # msg = Twist()
        # msg.angular.z = 0.1
        # msg.linear.x = 0.2
        # self.vel_pub.publish(msg)
        plt.scatter(self.points[:, 0], self.points[:, 1], c="r")
        plt.show()

    def laserCallback(self, msg):
        # rospy.loginfo("Resposta laser: " + str(msg.angle_increment))
        self.laser = msg.ranges
        self.angle_step = msg.angle_increment
        # rospy.loginfo("Laser len:" + str(len(self.laser)))
        self.points = np.zeros([1, 2])
        for i in range(len(self.laser)):
            self.point = np.array([self.pol_cart(self.laser[i], self.angle_step * i)])

            rospy.loginfo("ponto:" + str(self.point))
            # self.points = np.append(self.points, self.point, axis=1)
            rospy.loginfo("pontos:" + str(self.points))
            self.points = np.concatenate([self.point, self.points])

            # self.grid[self.point] = 1
            # rospy.loginfo("Laser:" + str(self.laser[i]))

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
