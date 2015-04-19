#!/usr/bin/env python

'''
    Controller node for HW5.

    Makes service calls to our CV node and tries to servo to a block to pick it up.

    It first servos to the bowl, nodding to indicate the bowl was found.
    It then servos to the block, retrieves it, and place it on the table.
'''

import rospy, robot_interface
from random import random
from sbb_hw5.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Hard-coded settings for initial pose and table location
        rospy.loginfo('Initializing controller')
        self.tableZ = -.07
        self.scanZ = .3
        self.safeZ = .08
        self.pixTol = 10
        self.thetaTol = .3
        self.bowlD = .15

        # Control gain includes the pixel to m conversion factor
        # At z = .08 (table distance .15), pixels = 78, m = .044
        # So multiply by .0013 to get m, then call gain ~ .8
        self.controlGain = .0013 * .8 * (self.safeZ - self.tableZ)/.15
        self.xmin = .3
        self.ymin = -.6
        self.xmax = .8
        self.ymax = -.1

        # Convient poses, orientations, etc
        self.downwards = Quaternion(1, 0, 0, 0)
        self.home = Pose(Point(.5, -.5, self.scanZ), self.downwards)
        self.destination = Pose(Point(.5, 0, self.tableZ), self.downwards)

        # initialize the Baxter object
        # Note that Baxter should be using his right arm
        # The bowl should be within the right arm's reachable workspace on table
        rospy.loginfo('Connecting to Baxter...')
        self.baxter = robot_interface.Baxter()
        self.baxter.enable()
        rospy.loginfo('Connected to Baxter successfully.')

        # Send Baxter's arm to the initial position
        self.goHome()

        # This node makes use of the find_stuff service
        self.cv = rospy.ServiceProxy('find_stuff', FindStuffSrv)
        rospy.loginfo('Initialization successful.')

    # This is the callback for processing incoming command messages.
    def goHome(self):
        self.baxter.setEndPose(self.home)

    # Get a response from CV service: found, x, y, theta (theta doesn't matter)
    def getBowl(self):
        rospy.wait_for_service('find_stuff')
        return self.cv('bowl')

    # Get a response from CV service: found, x, y, theta
    def getBlock(self):
        rospy.wait_for_servce('find_stuff')
        return self.cv('block')

    # Coordinates pick-place action done by Baxter
    # "from" and "to" need to be poses of the end link
    # Since we are AGAIN doing upright orientation only, neglect end effector
    def pickPlace(self, fromP, toP):

        rospy.loginfo('Performing pick/place.')

        # Make safe pre-pick pose and go
        ps = Pose(Point(fromP.x, fromP.y, fromP.z + self.safeZ), fromP.orientation)
        self.baxter.openGrip('right')
        self.baxter.setEndPose('right', ps)

        # Go to pick and pick
        self.baxter.setEndPose('right', fromP)
        self.baxter.closeGrip('right')

        # Path to place pose through safe points
        self.baxter.setEndPose('right', ps)
        ps = Pose(Point(toP.x, toP.y, toP.z + self.safeZ), toP.orientation)
        self.baxter.setEndPose('right', ps)
        self.baxter.setEndPose('right', toP)

        # Finish place and back off to safe height
        self.baxter.openGrip('right')
        self.baxter.setEndPose('right', ps)
        rospy.loginfo('Pick/place complete.')


    # This function attempts to put the end effector within sight of the bowl centroid.
    # If the current end pose does not accomplish this, it moves to a random pose in the 
    # search plane, and checks again - repeating until it is successful.
    # Never. Give. Up. Timeouts are for wimps.
    def scanForBowl(self):
        rospy.loginfo('Scanning for bowl...')
        while True:
            bowl = self.getBowl()
            if bowl.found:
                rospy.loginfo('Bowl found.')
                return bowl
            else:
                xpos = self.xmin + (self.xmax - self.xmin)*random()
                ypos = self.ymin + (self.ymax - self.ymin)*random()
                ps = Pose(Point(xpos, ypos, self.scanZ), self.downwards)
                self.baxter.setEndPose('right', ps)

    # Same as scanForBowl, except that the range of motion is restricted to the bowl's
    # diameter, about the initial point. Checks for exceeding xmin/xmax are implemented.
    def scanForBlock(self):
        rospy.loginfo('Scanning for block...')
        start = self.baxter.getEndPose('right')
        while True:
            block = self.getBlock()
            if block.found:
                rospy.loginfo('Block found.')
                return block
            else:
                xpos = start.position.x + self.bowlD*(random() - .5)
                ypos = start.position.y + self.bowlD*(random() - .5)
                ps = Pose(Point(xpos, ypos, self.scanZ), self.downwards)
                self.baxter.setEndPose('right', ps)

    # This function must be run only when we are reliably in range of the block.
    # It performs choppy closed-loop semi-proportional control of the end effector
    # until it is aligned and directly over the block.
    def controlToBlock(self):

        rospy.loginfo('Controlling to block center...')

        # Some setup: convenience function, limits, initial conditions
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        delTheta = 1000
        delY = 1000
        delX = 1000
        ps = self.baxter.getEndPose('right')
        ps.position.z = self.safeZ
        self.baxter.setEndPose('right', ps)

        # Control loop. Never. Give. Up.
        while True:
            ps = self.baxter.getEndPose('right')
            block = self.getBlock()
            delTheta = block.t
            delY = block.y
            delX = block.x
            if (delTheta > self.thetaTol) or (delY > self.pixTol) or (delX > self.pixTol):
                return ps
            newx = clamp(ps.position.x - (delX * self.controlGain), self.xmin, self.xmax)
            newy = clamp(ps.position.y - (delY * self.controlGain), self.ymin, self.ymax)
            newt = ps.orientation.w - .9*delTheta
            ps = Pose(Point(newx, newy, self.safeZ), Quaternion(1,0,0,newt))
            self.baxter.setEndPose(ps)

    # Performs the sequence of actions required to accomplish HW5 CV objectives.
    def execute(self):
        self.scanForBowl()
        self.scanForBlock()
        bPose = self.controlToBlock()
        bPose.position.z = self.tableZ
        self.pickPlace(bPose, self.destination)
        rospy.loginfo('All done!')

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        controller = Controller()
        controller.execute()
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)