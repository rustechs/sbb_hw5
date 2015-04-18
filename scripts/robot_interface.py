#!/usr/bin/env python

'''
    Robot Interface node for HW3.

     Query the position of the robot's arm and return it
    *Returns the configuration, and the cartesian pose of end-effector
    *Pick-and-place routine that accepts source and destination coordinates
'''

import argparse, sys, rospy, cv2, cv_bridge
import baxter_interface, rospkg

import roslib

from bax_hw3.msg import *

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Image

# The baxter class definition
# Acts as a wrapper for many useful baxter_interface methods
# Also spawns a node to interface with IK Service
class Baxter():

    # Baxter class constructor
    def __init__(self, baxter_name="Baxter"):

        rospy.init_node("Baxter_Node")
        
        # Give him a creative name
        self.name = baxter_name

        # self.enable() # Probably should be called manually
        
        # Create baxter arm instances
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')

        self.BaxEnable = baxter_interface.RobotEnable()

        # Create baxter gripper instances
        self.right_gripper = baxter_interface.Gripper('right')
        self.left_gripper = baxter_interface.Gripper('left')
        self.right_gripper.calibrate()
        self.left_gripper.calibrate()

        # Zero to wherever the left end is
        self.zero()

        # Set up publishing to the face
        rospack = rospkg.RosPack()
        self.impath = rospack.get_path('bax_hw3') + '/img/'
        self.facepub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

    # Tansformation from a local frame Pose to global frame
    def tfBaxter(self, wsPose):
        return Pose( Point(wsPose.position.x + self.zeroPose.position.x,
                           wsPose.position.y + self.zeroPose.position.y,
                           wsPose.position.z + self.zeroPose.position.z,),
                     wsPose.orientation)

    # Transformation from global frame to the local (zero'd) frame
    def tfBaxterInv(self, globPose):
        return Pose( Point(globPose.position.x - self.zeroPose.position.x,
                           globPose.position.y - self.zeroPose.position.y,
                           globPose.position.z - self.zeroPose.position.z,),
                     globPose.orientation)
            
    #The pose at calibration 0 point of our local working frame
    def zero(self, rel = None):
        if rel is None:
            self.zeroPose = self.getEndPose('left',raw=True)
        else:
            self.zeroPose = self.tfBaxter(rel)

    def face(self, fname):
        pass
        img = cv2.imread(self.impath + fname + '.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.facepub.publish(msg)

    # Enable the robot
    # Must be manually called after instantiation 
    def enable(self):
        self.BaxEnable.enable()

    # Disable the robot
    def disable(self):
        self.BaxEnable.RobotEnable.disable()

    # Check if robot is enabled
    def isEnabled(self):
        return self.BaxEnable._state.enabled
        
    # Stop the robot
    # Equivalent to hitting e-stop
    def stop(self):
        self.BaxEnable.stop()

    # Close specified gripper
    # Defaults to blocking
    def closeGrip(self, limbSide, block=True):
        try:
            if limbSide == 'left':
                self.left_gripper.close(block)
            elif limbSide == 'right':
                self.right_gripper.close(block)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Open specified gripper
    # Defaults to blocking
    def openGrip(self, limbSide, block=True):
        try:
            if limbSide == 'left':
                self.left_gripper.open(block)
            elif limbSide == 'right':
                self.right_gripper.open(block)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Set specified gripper's applied force
    # Specify force in % (0-100), mapping to 0-30 N
    def setGripForce(self, limbSide, force):
        try:
            if limbSide == 'left':
                self.left_gripper.set_moving_force(force)
                self.left_gripper.set_holding_force(force)
            elif limbSide == 'right':
                self.right_gripper.set_moving_force(force)
                self.right_gripper.set_holding_force(force)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper is ready
    # Returns true iff gripper is calibrated, not in error state, and not moving
    def getGripReady(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.ready()
            elif limbSide == 'right':
                return self.right_gripper.ready()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper is gripping (i.e. force threshold reached)
    def getGripGripping(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.gripping()
            elif limbSide == 'right':
                return self.right_gripper.gripping()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper missed object
    # (i.e. gripper closed without reaching force threshold)
    def getGripMissed(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.missed()
            elif limbSide == 'right':
                return self.right_gripper.missed()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise
    
    # Get specified gripper's current position
    # Returns as percent (0-100) of full travel range
    def getGripPos(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.position()
            elif limbSide == 'right':
                return self.right_gripper.position()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Get specified gripper's current applied force
    # Returns as percent (0-100) of max applicable force
    def getGripForce(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.force()
            elif limbSide == 'right':
                return self.right_gripper.force()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise
    
    # Method for getting joint configuration
    # Direct call to baxter_interface
    # Returns: dict({str:float})
    # unordered dict of joint name Keys to angle (rad) Values
    def getJoints(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_arm.joint_angles()
            elif limbSide == 'right':
                return self.right_arm.joint_angles()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name ' + limbSide)
            raise


    # Method for getting end-effector position
    # Angular pose will always be top-down, so wrist-gripper displacement doesn't have to be factored in.
    # Returns the raw ('base') Pose if raw is set to True
    # Otherwise, returns pose relative to the origin zeroPose
    def getEndPose(self,limbSide,raw=False):
        
        # Conveniently call Baxter's endpoint_pose() methods
        try:
            if limbSide == 'left':
                out = self.left_arm.endpoint_pose() 
            elif limbSide == 'right':
                out = self.right_arm.endpoint_pose()
            else: 
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + limbSide)
            raise

        # From dict to Pose object
        out = Pose(Point(*out['position']), Quaternion(*out['orientation']))
        if not raw:
            out = self.tfBaxterInv(out)
        return  out

    # Method for setting joint positions
    # Direct call to baxter_interface
    def setJoints(self,limbSide,angles):
        if limbSide == 'left':
            self.left_arm.move_to_joint_positions(angles)
        elif limbSide == 'right':
            self.right_arm.move_to_joint_positions(angles)
        else:
            rospy.logwarn('Incorrect limb string: %s' % limbSide)


    # Method for calculating joint angles given a desired end-effector pose
    def getIKGripper(self, limbSide, setPose):

        # Prepare the request
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ps = PoseStamped(header=hdr, pose=setPose,)
        ikreq = SolvePositionIKRequest([ps], [], 0)

        # Make the service call
        srvName = 'ExternalTools/' + limbSide + '/PositionKinematicsNode/IKService'
        srvAlias = rospy.ServiceProxy(srvName, SolvePositionIK)
        rospy.wait_for_service(srvName)
        resp = srvAlias(ikreq)

        # Get IK response and convert to joint position dict
        if (resp.isValid[0]):
            return dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            print("IK service: INVALID POSE - No Valid Joint Solution Found.")


    # Method for setting cartesian position of hand
    # setPose is a Pose relative to the home zeroPose
    def setEndPose(self, limbSide, setPose):
        setPose = self.tfBaxter(setPose)
        ik_joints = self.getIKGripper(limbSide, setPose)
        self.setJoints(limbSide,ik_joints)
