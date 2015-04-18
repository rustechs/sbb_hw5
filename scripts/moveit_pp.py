#!/usr/bin/env python

'''
    Quick MoveIt interface pick-place test script
    Accomplishes HW5 objective 2
    Works by specifying the block and goal positions by using cuff
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
        ps = g.get_current_pose()
        print ps.pose
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
        import pdb; pdb.set_trace()
        #g.set_pose_target(Pose(Point(x,y,z), Quaternion(xo,yo,zo,wo)))
        g.set_pose_target(Pose(Point(x,y,z),ps.pose.orientation))
        g.go()