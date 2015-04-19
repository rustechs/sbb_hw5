#!/usr/bin/env python

'''
    Controller node for HW5.

    Makes service calls to our CV node and tries to servo to a block to pick it up.

    It first servos to the bowl, nodding to indicate the bowl was found.
    It then servos to the block, retrieves it, and place it on the table.
'''

import rospy, robot_interface
from sbb_hw5.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Hard-coded settings for initial pose and table location
        rospy.loginfo('Initializing controller')
        self.tableZ = -.07
        self.scanZ = .2
        self.safeZ = .1
        self.downwards = Quaternion(1, 0, 0, 0)
        self.home = Pose(Point(.5, -.5, self.scanZ), self.downwards)
        self.destination = Pose(Point(.5, 0, self.tableZ), self.downwards)

        # Initialize progress trackers
        self.foundBowl = False
        self.foundBlock = False
        self.done = False

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


    # Function that takes action depending on the state of the workspace.
    # Coordination of arms happens external to this; high-level only.
    def planMove(self):

        # Start planning out move right away.

        # If the stack order matches the objective, express victory
        if not (None in [slot.contains for slot in self.stack]):
            order = [slot.contains.n for slot in self.stack]
            if order == self.objective:
                rospy.loginfo('Objective achieved.')
                self.baxter.face('happy')
                self.done = True
                return

        # If it is not done, travel up the stack looking for the first stack
        # slot that either contains the wrong block or is empty.
        for i, slot in list(enumerate(self.stack)):

            rightBlock = self.blocks[self.objective[i]]

            # If the slot is empty, place the block that belongs there.
            if slot.isEmpty():
                self.pickPlace(rightBlock, slot)
                return

            # If the slot contains the wrong block, take the top of the stack # off and place it on an available table slot.
            if not (slot.contains is rightBlock):
                nums = [self.stack[j].contains for j in range(self.nBlocks)]
                try:
                    indn = nums.index(None) - 1
                except:
                    indn = len(nums) - 1
                removeBlock = self.stack[indn].contains
                self.pickPlace(removeBlock, 'table')
                return

        rospy.loginfo('Planning could not find something to do ...?')

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        controller = Controller()
        while True:
            time.sleep(1)
            if not controller.done:
                controller.planMove();
                
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
