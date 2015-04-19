#!/usr/bin/env python

'''
    Robot Interface node for HW5.

    It has been altered since HW3 to use MoveIt.
    NOPE, JUST KIDDING - MoveIt doesn't work, so we're using the old
    baxter_interface tools. Perhaps with some speed tweaks.
'''

import sargparse, sys, rospy, cv2, cv_bridge
import baxter_interface, rospkg

from sbb_hw5.msg import *

from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Image

# The baxter class definition
# Acts as a wrapper for many useful baxter_interface, moveit methods
# Also spawns a node to interface with IK Service
class Baxter():

    # Baxter class constructor
    def __init__(self, baxter_name="Baxter", do_grippers = True):

        rospy.init_node("Baxter_Node")
        
        # Give him a creative name
        self.name = baxter_name
        
        # Create baxter arm instances
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')
        self.BaxEnable = baxter_interface.RobotEnable()

        # Create baxter gripper instances
        if do_grippers:
            self.right_gripper = baxter_interface.Gripper('right')
            self.left_gripper = baxter_interface.Gripper('left')
            self.right_gripper.calibrate()
            self.left_gripper.calibrate()

        # Create Baxter Hand Cameras
        self.rhc = baxter_interface.CameraController('right_hand_camera')
        self.lhc = baxter_interface.CameraController('left_hand_camera')

        # Set up publishing to the face
        rospack = rospkg.RosPack()
        self.impath = rospack.get_path('bax_hw3') + '/img/'
        self.facepub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)

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
    # Direct call to moveit
    # Returns: list of floats
    def getJoints(self, limbSide):
        try:
            if limbSide == 'left':
                return self.lg.get_current_joint_values()
            elif limbSide == 'right':
                return self.rg.get_current_joint_values()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name ' + limbSide)
            raise


    # Method for getting end-effector position
    def getEndPose(self, limbSide):
        
        # Conveniently call Baxter's endpoint_pose() methods
        try:
            if limbSide == 'left':
                return self.lg.get_current_pose() 
            elif limbSide == 'right':
                return self.rg.get_current_pose()
            else: 
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + limbSide)
            raise

    # Method for setting joint positions
    # Direct call to moveit, including go
    def setJoints(self,limbSide,angles):
        if limbSide == 'left':
            self.lg.set_joint_value_target(angles)
            self.lg.go()
        elif limbSide == 'right':
            self.rg.set_joint_value_target(angles)
            self.rg.go()
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
    # setPose is a global pose
    def setEndPose(self, limbSide, setPose):
        ik_joints = self.getIKGripper(limbSide, setPose)
        self.setJoints(limbSide,ik_joints)

    # Camera Settings
    def setCamera(self, name, res=(960,600), fps=10):
    # name = left, right
    # res = [(1280, 800), (960, 600), (640, 400), (480, 300), (384, 240), (320, 200)]
    # fps = frames per second
        try:
            if name == 'left':
                self.lhc.open(self)
                self.lhc.resolution(self, res)
                self.lhc.fps(self, fps)
                # self.lhc.gain(self, gain)                  # Camera gain. 
                # self.lhc.exposure(self, exposure)          # Camera Exposure. 
                # self.lhc.white_balance_red(self, value)    # White balance red. 
                # self.lhc.white_balance_green(self, value)  # White balance green. 
                # self.lhc.white_balance_blue(self, value)   # White balance blue.             
            elif name == 'right':
                self.rhc.open(self)
                self.rhc.resolution(self, res)
                self.rhc.fps(self, fps)
            else:
                raise
        except:
            rospy.logwarn('Invalid camera side name #: ' + name)
            raise
