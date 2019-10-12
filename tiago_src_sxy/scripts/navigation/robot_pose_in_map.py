#! /usr/bin/python
# -*- coding: utf-8 -*-
#
import rospy
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
import tf_lookup.srv as tls
import numpy
import os


class PoseRobot:
    def __init__(self, map_frame, base_frame):
        self.tf_lookup = rospy.ServiceProxy('/lookupTransform', tls.lookupTransform, persistent=True) #Global name, only one tf tree
        self.map_frame = map_frame
        self.base_frame = base_frame

    def robot_pose(self):
        # During localization (amcl running) we have the 'amcl_pose'
        # (including the covariance).
        # On the other side, during mapping we have the TF (/map -> /base_footprint).
        try:
            trans = self.tf_lookup(self.map_frame, self.base_frame, rospy.Time.from_sec(0))
        except rospy.ServiceException as exc:
            rospy.logwarn("Could not get tf from " + self.map_frame + " to " + self.base_frame + ": " + str(exc))
            return
        transform = trans.transform.transform
        pose = Pose(transform.translation, transform.rotation)
        print '\033c'# clear screen
        print 'robot pose:'
        print pose


def main():
    rospy.init_node("robot_pose_in_map")
    map_frame = rospy.get_param("~map_frame", "/map")
    base_frame = rospy.get_param("~base_frame", "/base_footprint")
    ps = PoseRobot(map_frame, base_frame)
    frequency = rospy.get_param("~frequency", 2.0)
    print "~map_fram", map_frame
    print "~base_frame", base_frame
    print "frequency", frequency
    r = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        ps.robot_pose()
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
