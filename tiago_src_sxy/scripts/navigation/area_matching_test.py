#! /usr/bin/python
# -*- coding: utf-8 -*-
#
import rospy
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
import tf_lookup.srv as tls
import numpy
import os
from shapely.geometry import Point
from shapely.geometry import Polygon


class matching_area:
    def __init__(self, dir_a):
        self.dir = dir_a

    def test_areas(self, P_x, P_y):
        for ar in self.dir:
            if self.test_one(self.dir[ar], P_x, P_y) == True:
                print '\033c'# clear screen
                print "The robot is in ", ar
        
    def test_one(self, vertex, x, y):
        area = Polygon(vertex)
        area_convex = area.convex_hull
        p_tmp = Point(x, y)
        #test
        # print(area_convex)
        # print "area", type(area)
        # print "area_convex", type(area_convex)
        # bbox = area_convex.bounds
        # min_x,min_y,max_x,max_y = bbox[0],bbox[1],bbox[2],bbox[3]
        # print "bonud:", min_x,min_y,max_x,max_y
        #
        return p_tmp.within(area_convex)


def area_matching(dir_a, P_x, P_y):
    for ar in dir_a:
        area = Polygon(dir_a[ar])
        area_convex = area.convex_hull
        p_tmp = Point(P_x, P_y)
        if p_tmp.within(area_convex) == True:
            print "The robot is in ", ar

class PoseRobot_area:
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
            return (0, 0)
        transform = trans.transform.transform
        pose = Pose(transform.translation, transform.rotation)
        # print "x:", pose.position.x
        # print "y:", pose.position.y
        return (pose.position.x, pose.position.y)




def main():
    rospy.init_node("robot_pose_in_map")
    map_frame = rospy.get_param("~map_frame", "/map")
    base_frame = rospy.get_param("~base_frame", "/base_footprint")
    ps = PoseRobot_area(map_frame, base_frame)
    frequency = rospy.get_param("~frequency", 8.0)
    # print "~map_fram", map_frame
    # print "~base_frame", base_frame
    # print "frequency", frequency
    r = rospy.Rate(frequency)
    # dictionary of areas
    dir_areas = {"room1":[(-4.7176,-12.2259), (4.5,-12.3232), (4.4945,-3.31011), (-4.561,-3.1376)], "room2":[(4.651, -2.811), (4.68641, 1.49968), (-4.4189, 1.636),(-4.3368, -2.4696)]}
    matching = matching_area(dir_areas)
    while not rospy.is_shutdown():
        loc_x, loc_y = ps.robot_pose()
        matching.test_areas(loc_x, loc_y)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
