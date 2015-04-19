#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class locate_stuff():

    def __init__(self):
        # Listen to camera image topic
        self.img_sub = rospy.Subscriber('cameras/right_hand_camera/image',Image,self.saveFrame)
        self.img_serv = rospy.Service('find_stuff',FindStuffSrv,self.servCall)
        
        # Create a image conversion bridge
        self.br = CvBridge()
        

    # Camera frame topic callback
    def saveFrame(self,data):
        try:
            self.img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

    # Deal with service call by parsing argument, figuring out what to look for
    def servCall(self,data):
        # Use latest image to look for stuff, return it
        if data.item == 'bowl':
            return FindStuffSrvResponse(*findBowl())
        elif data.item == 'block':
            return FindStuffSrvResponse(*findBlock())
        else
            raspy.logerr("Incorrect service call argument, use either bowl or block")
            raise

    def findBowl(self):

    def findBlock(self):


# Main loop
def main():
    # Instantiate a node
    locator = locate_stuff()
    # Initialize node
    rospy.init_node('locate_stuff', anonymous = True)
    
    print "Ready to find stuff!"

    # Wait for either a topic update or a service call
    rospy.spin()


if __name__ == '__main__':
    main()
