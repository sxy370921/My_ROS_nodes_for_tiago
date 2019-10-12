#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import copy

class laser_fusion():
    def __init__(self):
        # matched flag
        self.matched_flag = 0
        # data
        self.point_data = 0
        self.match_num = 0
        self.pub_data = LaserScan()
        # test
        # print type(self.pub_data.ranges) <type 'list'>
        # parameters
        self.scan_len = 0
        self.scan_point_len = 0
        self.matched_len = 0  # matched array length
        self.left_num = 0  # The matched laser data array first index got
        self.right_num = 0  # The matched laser data array last index got
        self.left_angle = 0  # The matched laser data array min angle got
        self.right_angle = 0  # The matched laser data array max angle got
        self.index_list = []  # The list describes the relationship between matched scan index and rgbd_scan index
        # init
        rospy.init_node('laser_fusion', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('/scan_fusion', LaserScan, queue_size=5)
        self.sub1 = rospy.Subscriber("/scan", LaserScan, self.callback_scan,queue_size=1)
        self.sub2 = rospy.Subscriber("/scan_point", LaserScan, self.callback_scan_point,queue_size=1)
        rate = 20
        r = rospy.Rate(rate)
        rospy.spin()

    def matched_list(self, data_scan):
        print '******index matching start*******'
        for num in range(self.left_num,(self.right_num+1)):
            # num and n are not only index of array but also interval of angle number
            angle1 = data_scan.angle_min + num * data_scan.angle_increment
            n = (angle1 - self.point_data.angle_min)/self.point_data.angle_increment
            print 'matched float num = ',n
            if n-int(n) < 0.5:
                n = int(n)
            else:
                n = int(n)+1
            if n > (self.scan_point_len-1):
                n = self.scan_point_len-1
            self.index_list.append(n)
            # test
            print 'pub_index=', num - self.left_num
            print 'matched rgbd_num=', n
            print 'matched scan_num=', num
            err = n * self.point_data.angle_increment + self.point_data.angle_min - angle1
            print 'error:', err,err*180/3.141592653589793
        print '******index matching end*******'

    def fusion_1(self, data_scan):
        # simple fusion methods---methods one
        # notice the meaning of : num,self.matched_len,n,num - self.left_num
        for num in range(self.left_num,(self.right_num+1)):
            # angle1 = data_scan.angle_min + num * data_scan.angle_increment
            # n = (angle1 - self.point_data.angle_min)/self.point_data.angle_increment
            # if n-int(n) < 0.5:
            #     n = int(n)
            # else:
            #     n = int(n)+1
            # if n > (self.scan_point_len-1):
            #     n = self.scan_point_len-1
            n = self.index_list[num - self.left_num]
            if (data_scan.ranges[num] - self.point_data.ranges[n]) > 0.1:
                self.pub_data.ranges[num - self.left_num] = self.point_data.ranges[n]
            else:
                self.pub_data.ranges[num - self.left_num] = data_scan.ranges[num]

            # test
            # print 'index', n

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        print "Stopping the fusion..."

    def scan_matching(self,data):
        self.match_num = (data.angle_max - self.point_data.angle_max) / data.angle_increment
        self.match_num = self.match_num + 1
        print "match_num float= ", self.match_num
        # # notice: turn to a smaller int number
        # self.match_num = int(self.match_num)
        # notice: turn to a larger int number
        self.match_num = int(self.match_num) + 1
        print "match_num int= ", self.match_num
        self.scan_len = len(data.ranges)
        self.scan_point_len = len(self.point_data.ranges)
        print "laser array len = ", self.scan_len, "Point_laser array len = ", self.scan_point_len
        self.left_num = self.match_num - 1
        self.right_num = self.scan_len - self.match_num
        self.left_angle = data.angle_min + self.left_num * data.angle_increment
        self.right_angle = data.angle_max - self.left_num * data.angle_increment
        self.matched_len = self.right_num - self.left_num + 1
        print "matched left_num = ", self.left_num, 'matched left_angle=', self.left_angle
        print "matched right_num = ", self.right_num, 'matched right_angle=', self.right_angle
        print 'matched array len=', self.matched_len, '=', len(data.ranges[self.left_num:(self.right_num + 1)])
        print 'rgbd_ranges angle_min=', self.point_data.angle_min, 'rgbd_ranges angle_max=', self.point_data.angle_max
        print 'scan angle_min=', data.angle_min, 'scan angle_max=', data.angle_max
        print '*******Matched successfully**********'
        # setting every parameters of pub_data
        self.pub_data.ranges = [0 for x in range(self.matched_len)]  # give the list right index
        self.pub_data.header.frame_id = data.header.frame_id
        self.pub_data.angle_min = self.left_angle
        self.pub_data.angle_max = self.right_angle
        self.pub_data.angle_increment = data.angle_increment
        print 'len pub_ranges', len(self.pub_data.ranges)  # test
        if data.time_increment > self.point_data.time_increment:
            self.pub_data.time_increment = data.time_increment
        else:
            self.pub_data.time_increment = self.point_data.time_increment
        if data.scan_time > self.point_data.scan_time:
            self.pub_data.scan_time = data.scan_time
        else:
            self.pub_data.scan_time = self.point_data.scan_time
        self.pub_data.range_min = data.range_min
        self.pub_data.range_max = data.range_max
        self.matched_list(data)  # generate corresponding index between matched scan and RGBD scan

    def callback_scan(self, data):
        # The first session is range matching
        if self.matched_flag == 0:
            if self.point_data != 0:
                self.matched_flag = 1
                self.scan_matching(data)
            else:
                print "*******Not received Point_laser!*******"
        else:
            # The second session is time matching
            if (data.header.stamp.secs == self.point_data.header.stamp.secs) and \
                    (abs(data.header.stamp.secs - self.point_data.header.stamp.secs) < 300000000):
                # The third session is fusion
                self.pub_data.header.stamp = rospy.get_rostime()
                self.fusion_1(data)
                print "******time matching is successful******"
                # test
            else:
                self.pub_data.header.stamp = rospy.get_rostime()
                print "******time matching is wrong******"
            self.pub.publish(self.pub_data)

        # test
        if self.matched_flag == 1:
            pass
            # print "matched scan data:\n",data.ranges[self.left_num:(self.right_num+1)]

    def callback_scan_point(self, data):
        self.point_data = copy.deepcopy(data)

        # test
        # print "Point_laser data:\n",self.point_data.ranges


if __name__ == '__main__':
    try:
        laser_fusion()
    except:
        rospy.loginfo("laser_fusion node terminated.")
