#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Explorer:
    def __init__(self):

        self.origin = [0, 0]
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

    def timerCallback(self, event):
        msg = Twist()
        msg.angular.z = 0.1
        msg.linear.x = 0.2
        self.vel_pub.publish(msg)

    def laserCallback(self, msg):
        # rospy.loginfo("Resposta laser: " + str(msg.angle_increment))
        self.laser = msg.intensities
        self.angle = msg.angle_increment
        print(type(self.laser))
        # print(len(self.laser))
        # for i in range(len(self.laser)):
        #     self.rad[i] = i * self.angle

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
