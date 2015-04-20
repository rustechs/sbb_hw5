#!/usr/bin/env python

'''
    Controller node for HW5.

    Makes service calls to our CV node and tries to servo to a block to pick it up.

    It first servos to the bowl, nodding to indicate the bowl was found.
    It then servos to the block, retrieves it, and place it on the table.
'''

import rospy, robot_interface
from random import random
from math import pi
from sbb_hw5.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Hard-coded settings for initial pose and table location
        rospy.loginfo('Initializing controller')
        self.tableZ = -.07
        self.scanZ = .3
        self.safeZ = .08
        self.xyTol = .005
        self.thetaTol = .3
        self.bowlD = .15
        self.controlGain = .5
        self.pixCalZ = .15
        self.pixCalN = 78
        self.pixCalD = .044
        self.camOffsetX = .035
        self.camOffsetY = 0
        self.xmin = .3
        self.ymin = -.6
        self.xmax = .8
        self.ymax = -.1

        # Convient poses, orientations, etc
        self.downwards = self.e2q(0, pi, 0)
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

    # Convenience function for euler_to_quaternion that returns as Quaternion.
    def e2q(self, x, y, z):
        q = tuple(quaternion_from_euler(x, y, z))
        return Quaternion(*q)

    # Convenience function for getting the angular alignment in YZ plane from pose
    # For some reason, euler_from_quaternion is phased by pi relative to quaternion_from_euler
    def getW(self, p):
        eu = euler_from_quaternion([p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w])
        return self.rerange(eu[2] + pi)

    # Convenience function for changing the angular alignment only
    def setW(self, w):
        self.baxter.setEndXYZO('right', o = self.e2q(0, pi, w))

    # Puts a value in the -pi to pi range
    def rerange(self, val):
        if val > pi:
            return (val % (2 * pi)) - 2 * pi
        elif val < -pi:
            return -(-val % (2 * pi)) + 2 * pi
        else:
            return val

    # This function converts pixels to meters on the table plan given a height.
    # It assumes/requires a downward orientation.
    # It takes care of checking the height off the table itself.
    def pix2m(self, pix):
        ps = self.baxter.getEndPose('right')
        height = ps.position.z - self.tableZ
        return (pix * (height/self.pixCalZ) * self.pixCalD) / self.pixCalN

    # This function send Baxter to its hard-coded "home" position.
    def goHome(self):
        self.baxter.setEndPose('right', self.home)

    # Get a response from CV service: found, x, y, theta (theta doesn't matter)
    # x, y are converted to meters before returning
    # This REQUIRES home orientation (orientation is self.downwards)
    def getBowl(self):
        rospy.wait_for_service('find_stuff')
        bowl = self.cv('bowl')
	oldx = bowl.x
        bowl.x = self.pix2m(bowl.y) - self.camOffsetX
        bowl.y = -self.pix2m(oldx) - self.camOffsetY
        return bowl

    # Get a response from CV service: found, x, y, theta
    # x, y are converted to meters before returning
    # This REQUIRES home orientation (orientation is self.downwards)
    def getBlock(self):
        rospy.wait_for_service('find_stuff')
        block = self.cv('block')
	oldx = block.x
        block.x = self.pix2m(block.y) - self.camOffsetX
        block.y = -self.pix2m(oldx) - self.camOffsetY
        return block

    # Coordinates pick-place action done by Baxter
    # "from" and "to" need to be poses of the end link
    # Since we are AGAIN doing upright orientation only, neglect end effector
    def pickPlace(self, fromP, toP):
        rospy.loginfo('Performing pick/place.')

        # Make safe pre-pick pose and go
        self.baxter.openGrip('right')
        self.baxter.setEndXYZO('right', fromP.position.x, fromP.position.y, self.safeZ, fromP.orientation)

        # Go to pick and pick
        self.baxter.setEndPose('right', fromP)
        self.baxter.closeGrip('right')

        # Path to place pose through safe points
        self.baxter.setEndXYZO('right', fromP.position.x, fromP.position.y, self.safeZ, fromP.orientation)
        self.baxter.setEndXYZO('right', toP.position.x, toP.position.y, self.safeZ, toP.orientation)

        # Finish place and back off to safe height
        self.baxter.setEndPose('right', toP)
        self.baxter.openGrip('right')
        self.baxter.setEndXYZO('right', toP.position.x, toP.position.y, self.safeZ, toP.orientation)
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
                ps = Pose(Point(xpos, ypos, self.safeZ), self.downwards)
                self.baxter.setEndPose('right', ps)

    # This function must be run only when we are reliably in range of an object
    # It performs choppy closed-loop semi-proportional control of the end effector
    # until it is aligned and directly over the object.
    # THIS REQUIRES HOME ORIENTATION - I.E., orientation is self.downwards
    # In fact, this condition is enforced if it does not already exist.
    def controlTo(self, objective):
        rospy.loginfo('Controlling to %s center...' % objective)

        # Some setup: convenience function, limits, initial conditions
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        delY = 1000
        delX = 1000
        self.setW(0)

        # Select objective function
        if objective == 'block':
            objfun = lambda: self.getBlock()
        else:
            objfun = lambda: self.getBowl()

        # Control loop. Never. Give. Up.
        while True:
            ps = self.baxter.getEndPose('right')
            obj = objfun()
            if not obj.found:
                rospy.loginfo('Lost %s! Aborting control.' % objective)
                return  
            delY = abs(obj.y)
            delX = abs(obj.x)
            if (delY < self.xyTol) and (delX < self.xyTol):
                rospy.loginfo('Center control complete.')                
                return ps
            newx = clamp(ps.position.x + (obj.x * self.controlGain), self.xmin, self.xmax)
            newy = clamp(ps.position.y + (obj.y * self.controlGain), self.ymin, self.ymax)
            self.baxter.setEndXYZO('right', x = newx, y = newy, o = self.downwards) 

    # This does the rotational alignment control.
    # It requires that the gripper already be centered (controlTo) on the item.
    # This only has meaning for block, so it takes no argument.
    def alignWithBlock(self):
        rospy.loginfo('Aligning with block ...')
        delTheta = 1000

        # Control loop. Never. Give. Up.
        while True:
            ps = self.baxter.getEndPose('right')
            obj = self.getBlock()
            if not obj.found:
                rospy.loginfo('Lost block! Aborting control.')
                return  
            delTheta = abs(obj.t)
            if (delTheta < self.thetaTol):
                rospy.loginfo('Alignment complete.')                
                return ps
            newt = self.rerange(self.getW(ps)- obj.t)
            self.setW(newt)

    # Performs the sequence of actions required to accomplish HW5 CV objectives.
    def execute(self):
        try:
            self.scanForBowl()
            self.controlTo('bowl')
            self.baxter.setEndXYZO('right', z = self.safeZ)
            self.scanForBlock()
            self.controlTo('block')
            bPose = self.alignWithBlock()
            bPose.position.z = self.tableZ
            self.pickPlace(bPose, self.destination)
            rospy.loginfo('All done!')
        except:
            rospy.loginfo('Something went wrong while executing...')
            raise

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
