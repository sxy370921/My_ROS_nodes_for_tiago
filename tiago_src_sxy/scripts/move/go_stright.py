#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        rospy.init_node('move_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        rate = 20
        r = rospy.Rate(rate)
        linear_speed = 0.03
        goal_distance = 1.0
        linear_duration = goal_distance / linear_speed
        angular_speed = 1.0

        goal_angle = pi
        angular_duration = goal_angle / angular_speed
        move_cmd = Twist()
        while not rospy.is_shutdown():
            move_cmd.linear.x = linear_speed
            print("linear:", move_cmd.linear.x, "angular:", move_cmd.angular.z)
            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")

if __name__ == '__main__':
    try:
        OutAndBack()
    except rospy.ROSInterruptException:
        rospy.loginfo("Out-and-Back node terminated.")
