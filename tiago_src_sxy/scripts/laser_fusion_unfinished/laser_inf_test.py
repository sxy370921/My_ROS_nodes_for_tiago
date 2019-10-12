#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class laser_inf():
    def __init__(self):
        self.if_inf = 0
        self.beg = 0
        self.data_len = 0
        self.pub_data = LaserScan()
        rospy.init_node('laser_inf', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('/scan_inf', LaserScan, queue_size=5)
        self.sub2 = rospy.Subscriber("/rgbd_scan", LaserScan, self.callback_inf,queue_size=1)
        self.sub1 = rospy.Subscriber("/if_flag", Bool, self.callback_confirm, queue_size=1)
        rospy.spin()

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        print "Stopping the inf..."

    def callback_confirm(self,data):
        self.if_inf = data.data

    def callback_inf(self, data):
        if self.beg == 0:
            self.data_len = len(data.ranges)
            self.pub_data = data
            self.pub_data.ranges = [0 for x in range(self.data_len)]
            self.beg = 1
        if self.if_inf == True:
            i = int(self.data_len/4)
            for g in range(self.data_len):
                self.pub_data.ranges[g] = data.ranges[g]
            for k in range(i):
                self.pub_data.ranges[k] = float("inf")
            for h in range((2 * i - 1),(3 * i)):
                self.pub_data.ranges[h] = float("inf")
            self.pub_data.header.stamp = rospy.get_rostime()
            self.pub.publish(self.pub_data)
        else:
            data.header.stamp = rospy.get_rostime()
            self.pub.publish(data)




if __name__ == '__main__':
    try:
        laser_inf()
    except:
        rospy.loginfo("laser_inf node terminated.")
