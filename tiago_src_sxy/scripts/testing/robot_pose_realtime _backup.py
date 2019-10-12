#! /usr/bin/python
# -*- coding: utf-8 -*-
#


import rospy

from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped


import tf.transformations as tft
import tf_lookup.srv as tls

import numpy
import yaml

import os

def default_covariance():
  covariance = numpy.zeros([6, 6])
  covariance[0, 0] = 0.01
  covariance[1, 1] = 0.01
  covariance[5, 5] = 0.02

  return covariance

def pose_to_params(pose, covariance):
  params = {}

  # Take yaw from orientation quaternion:
  q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
  (_, _, yaw) = tft.euler_from_quaternion(q)

  params["initial_pose_x"] = pose.position.x
  params["initial_pose_y"] = pose.position.y
  params["initial_pose_a"] = yaw

  params["initial_cov_xx"] = numpy.asscalar(covariance[0, 0])
  params["initial_cov_yy"] = numpy.asscalar(covariance[1, 1])
  params["initial_cov_aa"] = numpy.asscalar(covariance[5, 5])

  return params

class PoseSaverException(Exception):
  pass

class PoseRobot:

  def __init__(self, map_frame, base_frame):
    # Get params:
    self.tf_lookup = rospy.ServiceProxy('/lookupTransform', tls.lookupTransform, persistent=True) #Global name, only one tf tree
    self.map_frame = map_frame
    self.base_frame = base_frame

    # Subscribe to 'pal_navigation/state' to know if were in LOC or MAP state.
    # We avoid using pal_common.is_node_running('amcl') because it creates a
    # socket to query the ROS master.


    # Subscribe to 'amcl_pose' since it provides the covariance,
    # although it's not available during mapping
    self._amcl_pose = None
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)



  def _pose_callback(self, msg):
    self._amcl_pose = msg.pose


  def robot_pose(self):
    # During localization (amcl running) we have the 'amcl_pose'
    # (including the covariance).
    # On the other side, during mapping we have the TF (/map -> /base_footprint).
    # try:
    #     trans = self.tf_lookup(self.map_frame, self.base_frame, rospy.Time.from_sec(0))
    # except rospy.ServiceException as exc:
    #     rospy.logwarn("Could not get tf from " + self.map_frame + " to " + self.base_frame + ": " + str(exc))
    #     return
    # transform = trans.transform.transform
    # pose = Pose(transform.translation, transform.rotation)
    # print '---'
    # print 'robot pose_test:'
    # print pose
    if self._amcl_pose is not None:
      pose = self._amcl_pose.pose
      #vector turn to maxtri without change in content
      covariance = numpy.reshape(self._amcl_pose.covariance, [6, 6])
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
      print 'robot pose:'
      print pose
      print '---'



def main():
  rospy.init_node("robot_pose_realtime")

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
