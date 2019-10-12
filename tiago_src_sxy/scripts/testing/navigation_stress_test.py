#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import random
import std_srvs.srv

GOAL_POSE = {'C1': [3.950146, 0.849521, 0.000],
             'S4': [3.734791, -0.454118, 0.000],
             'S3': [3.297905, 0.179620, 3.141592],
             'S2': [1.891297, 0.872936, 3.141592],
             'S1': [1.732864, -0.622901, -3.141592],
             'S5': [1.971264, 0.144555, 0.000],
             'EXIT': [0.203432, -0.024925, 0.000],
             'EW': [0.117365, 0.727361, 1.570796],
             'EE': [-0.047621, -0.969630, -1.570796]}


class NavigationStressTest(object):
    def __init__(self):
        self.event_in = None
        self.nav_goal = None
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)
    
    def run(self):        
        while not rospy.is_shutdown():
            self.publish_random_goal()
            rospy.sleep(10)
            rospy.sleep(10)
            
            

    def publish_random_goal(self):
        goal = random.choice(list(GOAL_POSE.keys()))
        pose = GOAL_POSE[goal]
        self.nav_goal = geometry_msgs.msg.PoseStamped()
        self.nav_goal.header.frame_id = '/map'
        self.nav_goal.pose.position.x = pose[0]
        self.nav_goal.pose.position.y = pose[1]
        self.nav_goal.pose.orientation.z = pose[2]
        self.nav_pub.publish(self.nav_goal)
        rospy.loginfo("going to pose "+goal)
       

def main():
    rospy.init_node("nav_goal_stresstest", anonymous=False)
    nav_stress_test = NavigationStressTest()
    nav_stress_test.run()

if __name__ == '__main__':
    main()