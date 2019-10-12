#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class flag():
    def __init__(self):
        rospy.init_node('flag', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        goal_cmd = PoseStamped()
        goal_cmd.header.frame_id = "map"
        goal_cmd.header.stamp.secs = rospy.get_rostime().secs
        goal_cmd.header.stamp.nsecs = rospy.get_rostime().nsecs
        #goal_cmd.header.stamp = rospy.get_rostime()
        goal_cmd.pose.position.x=0.668555617332
        goal_cmd.pose.position.y=-5.95182037354
        goal_cmd.pose.position.z=0.0
        goal_cmd.pose.orientation.x=0.0
        goal_cmd.pose.orientation.y=0.0
        goal_cmd.pose.orientation.z=0.675359197265
        goal_cmd.pose.orientation.w=0.737488952235
        if not rospy.is_shutdown():
            self.pub.publish(goal_cmd)
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        print "Press any key to continue"

if __name__ == '__main__':
    try:
        flag()
    except rospy.ROSInterruptException :
        pass

