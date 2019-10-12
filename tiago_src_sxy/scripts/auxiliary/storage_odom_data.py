#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class collection():
    def __init__(self):
        self.num = 0
        self.name = raw_input('file name:')
        self.name = '/home/zy/data/' + self.name + '.txt'
        print self.name
        print 'velovity.x  velovity.y  velovity.angle  position.x position.y '
        self.f = open(self.name, 'w')
        rospy.init_node('collection', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.sub1 = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.callback, queue_size=1)
        # self.sub2 = rospy.Subscriber("/scan_point", LaserScan, self.callback_scan_point,queue_size=1)
        rospy.spin()

    def fusion(self):
        pass

    def shutdown(self):
        self.f.close()
        # Always stop the robot when shutting down the node.
        rospy.loginfo("collection the fusion...")

    def callback(self, data):
        if self.num == 10:
            self.num = 0
            a = '{:.6f}'.format(data.twist.twist.linear.x) + '    ' + '{:.6f}'.format(data.twist.twist.linear.y) + '    ' + '{:.6f}'.format(data.twist.twist.angular.z) + '    ' + '{:.6f}'.format(data.pose.pose.position.x) + '    ' + '{:.6f}'.format(data.pose.pose.position.y) + '\n'
            self.f.write(a)

        self.num = self.num + 1

if __name__ == '__main__':
    try:
        collection()
    except:
        self.f.close()
        rospy.loginfo("collection.")
