#!/usr/bin/env python

import rospy
import pprint as pp
from sensor_msgs.msg import LaserScan


class laser_fusion():
    def __init__(self):
        rospy.init_node('laser_fusion', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        #self.pub = rospy.Publisher('/scan_fusion', LaserScan, queue_size=5)
        self.sub1 = rospy.Subscriber("/scan", LaserScan, self.callback_scan,queue_size=1)
        self.sub2 = rospy.Subscriber("/scan_point", LaserScan, self.callback_scan_point,queue_size=1)
        rate = 20
        r = rospy.Rate(rate)
        rospy.spin()

    def fusion(self):
        pass

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the fusion...")

    def callback_scan(self, data):
        pub_data = LaserScan()
        pp.pprint(len(data.ranges))
        #pp.pprint(rospy.get_rostime())
        #pub_data.header.stamp=rospy.get_rostime()
        #self.pub.publish(pub_data)

    def callback_scan_point(self, data):
        pp.pprint(len(data.ranges))


if __name__ == '__main__':
    try:
        laser_fusion()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
