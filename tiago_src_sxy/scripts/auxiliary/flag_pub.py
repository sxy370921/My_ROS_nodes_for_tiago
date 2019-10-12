#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class flag():
    def __init__(self):
        rospy.init_node('flag', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('/if_flag', Bool, queue_size=5)
        while not rospy.is_shutdown():
            a = input('input:')
            if a == 1:
                self.pub.publish(True)
            if a == 2:
                self.pub.publish(False)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        print "Press any key to continue"

if __name__ == '__main__':
    try:
        flag()
    except rospy.ROSInterruptException :
        pass

