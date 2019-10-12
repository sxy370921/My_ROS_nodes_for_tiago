#! /usr/bin/python
# -*- coding: utf-8 -*-
#
import rospy
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
import tf_lookup.srv as tls
import numpy
import os


def default_covariance():
    covariance = numpy.zeros([6, 6])
    covariance[0, 0] = 0.01
    covariance[1, 1] = 0.01
    covariance[5, 5] = 0.02
    return covariance


class PoseRobot:
    def __init__(self, map_frame, base_frame):
        self.tf_lookup = rospy.ServiceProxy('/lookupTransform', tls.lookupTransform, persistent=True) #Global name, only one tf tree
        self.map_frame = map_frame
        self.base_frame = base_frame
        self._amcl_pose = None
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)
        # 不知道为什么这个接收器没有用spin仍然能接收数据
        # 不知为什么amcl提供的位姿和机器人basefoot在map上的位姿有差别，并且机器人basefoot在map上的位姿在一直变而amcl只有机器人动才变
    def _pose_callback(self, msg):
        self._amcl_pose = msg.pose

    def robot_pose(self):
        if self._amcl_pose is not None:
            pose = self._amcl_pose.pose
            #vector turn to maxtri without change in content
            covariance = numpy.reshape(self._amcl_pose.covariance, [6, 6])
            print '\033c'
            print 'robot pose:'
            print pose
            print 'covariance of estimation:'
            print covariance
            
        else:
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
    # During localization (amcl running) we have the 'amcl_pose'
    # (including the covariance).
    # On the other side, during mapping we have the TF (/map -> /base_footprint).

def main():
    rospy.init_node("robot_loc_data")
    map_frame = rospy.get_param("~map_frame", "/map")
    base_frame = rospy.get_param("~base_frame", "/base_footprint")
    ps = PoseRobot(map_frame, base_frame)
    frequency = rospy.get_param("~frequency", 1.0)
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
        print 'failure'
