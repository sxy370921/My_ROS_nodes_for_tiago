#! /usr/bin/python
# -*- coding: utf-8 -*-
#
# pose_saver.py
#
# Copyright (c) 2012-2013 PAL Robotics SL. All Rights Reserved
#
# Authors:
#   * Enrique Fernández

import rospy

from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
from pal_navigation_msgs.msg import NavigationStatus

from pal_python import pal_common

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

class PoseSaver:

  def __init__(self, map_frame, base_frame):
    # Get params:
    self._filename = os.path.join(os.getenv("HOME"), ".pal/pose.yaml")
    self._filename = rospy.get_param("~file", self._filename)
    self.tf_lookup = rospy.ServiceProxy('/lookupTransform', tls.lookupTransform, persistent=True) #Global name, only one tf tree
    self.map_frame = map_frame
    self.base_frame = base_frame

    # Subscribe to 'pal_navigation/state' to know if were in LOC or MAP state.
    # We avoid using pal_common.is_node_running('amcl') because it creates a
    # socket to query the ROS master.
    self._state = None
    rospy.Subscriber("pal_navigation_sm/state", NavigationStatus, self._state_callback, queue_size=1)

    # Subscribe to 'amcl_pose' since it provides the covariance,
    # although it's not available during mapping
    self._last_pose = None
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)

  def __del__(self):
    # Revert file if running in simulation:
    if pal_common.is_simulation():
      self._save_default_pose()

  def _state_callback(self, msg):
    self._state = msg.status.data

  def _pose_callback(self, msg):
    self._last_pose = msg.pose

  def _save_default_pose(self):
    pose = Pose()
    pose.orientation.w = 1.0

    self._save_pose(pose, default_covariance())

  def _save_pose(self, pose, covariance):
    # Serialize params to YAML:
    params = pose_to_params(pose, covariance)
    params_str = yaml.dump(params)

    # Open file (unbuffered):
    try:
      folder = os.path.dirname(self._filename)
      if not os.path.exists(folder):
        rospy.logwarn("Folder %s doesn't exist! Creating it." % folder)
        os.makedirs(folder)
      self._file = open(self._filename, "w", 0)
    except IOError:
      rospy.logfatal("Failed to open file %s!" % self._filename)
      raise PoseSaverException("file not found")
    except OSError:
      rospy.logfatal("Failed to create folder for file %s!" % self._filename)
      raise PoseSaverException("folder not found")

    # Save params:
    self._file.write(params_str)

    # Close file:
    self._file.close()

  def save_pose(self):
    # During localization (amcl running) we have the 'amcl_pose'
    # (including the covariance).
    # On the other side, during mapping we have the TF (/map -> /base_footprint).
    if self._state == 'LOC' and self._last_pose is not None:
      pose = self._last_pose.pose

      covariance = numpy.reshape(self._last_pose.covariance, [6, 6])
    else:
      try:
          trans = self.tf_lookup(self.map_frame, self.base_frame, rospy.Time.from_sec(0))
      except rospy.ServiceException as exc:
          rospy.logwarn("Could not get tf from " + self.map_frame + " to " + self.base_frame + ": " + str(exc))
          return

      transform = trans.transform.transform

      pose = Pose(transform.translation, transform.rotation)

      covariance = default_covariance()

      self._last_pose = PoseWithCovariance()
      self._last_pose.pose = pose
      self._last_pose.covariance = covariance

    # Save pose:
    self._save_pose(pose, covariance)

def main():
  rospy.init_node("pose_saver")

  map_frame = rospy.get_param("~map_frame", "/map")
  base_frame = rospy.get_param("~base_frame", "/base_footprint")

  ps = PoseSaver(map_frame, base_frame)

  frequency = rospy.get_param("~frequency", 1.0)

  r = rospy.Rate(frequency)
  while not rospy.is_shutdown():
      ps.save_pose()
      r.sleep()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
