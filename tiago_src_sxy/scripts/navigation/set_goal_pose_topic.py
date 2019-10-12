#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from geometry_msgs.msg import PoseStamped

class SetAnGoal():
    def __init__(self):
        rospy.init_node('set_goal', anonymous=True)
        self.cmd_vel = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=2)
        self.goal_cmd = PoseStamped()
        self.goal_cmd.header.frame_id = "map"
    def go(self):
        self.goal_cmd.header.stamp.secs = rospy.get_rostime().secs
        self.goal_cmd.header.stamp.nsecs = rospy.get_rostime().nsecs
        #goal_cmd.header.stamp = rospy.get_rostime()
        self.goal_cmd.pose.position.x=0.163424730301
        self.goal_cmd.pose.position.y=-1.98231506348
        self.goal_cmd.pose.position.z=0.0
        self.goal_cmd.pose.orientation.x=0.0
        self.goal_cmd.pose.orientation.y=0.0
        self.goal_cmd.pose.orientation.z=-0.836906673362
        self.goal_cmd.pose.orientation.w=0.54734561301
        self.cmd_vel.publish(self.goal_cmd)
        print "ok"
        rospy.sleep(1)
        self.cmd_vel.publish(self.goal_cmd)
        print "ok"
        rospy.sleep(1)
        # 不知道为什么第一次无法发布到话题上，必须第二次才行。并且中间要间隔时间，不间隔时间发两次也发不到话题上
        
        #OR
        # while not rospy.is_shutdown():
        #     self.cmd_vel.publish(self.goal_cmd)
        #     print "ok"
        #     rospy.sleep(2)


if __name__ == '__main__':
    try:
        s = SetAnGoal()
        s.go()
    except:
        rospy.loginfo("set_goal node terminated.")
