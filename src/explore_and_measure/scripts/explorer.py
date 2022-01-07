#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan


class Explorer:
    def __init__(self):
        
        self.laser_sub = rospy.Subscriber("/front/scan", LaserScan, self.laserCallback, queue_size=10)

    def laserCallback(self, msg):
        rospy.loginfo("Resposta laser: " + str(msg.angle_increment))


if __name__ =='__main__':
    try:
        rospy.init_node("explorer")
        Explorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass