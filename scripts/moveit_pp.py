#!/usr/bin/env python

'''
    Quick MoveIt interface pick-place test script
    Accomplishes HW5 objective 2
'''

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander

' Main execution '
if __name__ == '__main__':
    rospy.init_node('moveit_pp')
    g = MoveGroupCommander("left_arm")
    g.set_planner_id('RRTConnectkConfigDefault')

    while True:
        print 'Current pose is '
        p = g.get_current_pose()
        print p.pose
        print ''
        print 'new position:'
        x = input('x: ')
        y = input('y: ')
        z = input('z: ')
        print 'new orientation:'
        #xo = input('x: ')
        #yo = input('y: ')
        #zo = input('z: ')
        #wo = input('w: ')

        #g.set_pose_target(Pose(Point(x,y,z), Quaternion(xo,yo,zo,wo)))
        g.set_pose_target(Pose(Point(x,y,z),p.pose.orientation))
        g.go()