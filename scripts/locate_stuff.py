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
        self.img_sub = rospy.Subscriber('cameras/right_hand_camera/image',Image,self.parseFrame)
        # Provide a service to return newest bowl/block pose
        self.img_serv = rospy.Service('find_stuff',FindStuffSrv,self.servCall)
        # Publish overlayed image
        self.img_pub = rospy.Publisher('/robot/xdisplay',Image)
        # Create a image conversion bridge
        self.br = CvBridge()
        self.blockLoc = (False,0,0,0)
        self.bowlLoc = (False,0,0,0)
        self.center = (0,0)

    # Camera frame topic callback
    # Find bowl and block on each frame refresh
    def parseFrame(self,data):
        # Convert image into openCV format
        try:
            self.img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
            raise
        
        # Get frame center
        self.center = (int(self.img.shape[0]/2),int(self.img.shape[1]/2))

        # Convert image to HSV
        self.imgHSV = cv2.cvtcolor(img,cv2.COLOR_BGR2HSV)

        # Detect block and bowl, save to state variables
        self.blockLoc = self.findBlock()
        self.bowlLoc = self.findBowl()
        
        # Publish a pretty picture of what we found
        self.pubOverlayImg()

    def pubOverlayImg():

        # If block was found, show it
        if blockLoc[0]:
            cv2.circle(self.img,self.blockLoc[1:2],5,(30,50,240),-1)            
        # If bowl was found, show it
        if bowlLoc[0]:
            cv2.circle(self.img,self.bowlLoc[1:2],5,(30,240,30),-1)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
            
    # Deal with service call by parsing argument, returning latest location for item
    def servCall(self,data):
        # Use latest image to look for stuff, return it
        if data.item == 'bowl':
            return FindStuffSrvResponse(*self.bowlLoc)
        elif data.item == 'block':
            return FindStuffSrvResponse(*self.blockLoc)
        else
            raspy.logerr("Incorrect service call argument, use either bowl or block")
            raise

    # Find bowl using basic color thresholding centroid method
    # If found, orientation defaults to 0
    # Returns type (found,x,y,t)
    def findBowl(self):
        MIN = np.array([25,60,10])
        MAX = np.array([85,255,255])

        imgThresh = cv2.inRange(self.imgHSV,MIN,MAX)

        # Check if there's enough green pixels to constitute a bowl
        if float(cv2.countNonZero(imgThresh))/(self.img.shape[0]*self.img.shape[1]) >= 0.1:
            m = cv2.moments(imgThresh)
            x = int(m['m10']/m['m00'])
            y = int(m['m01']/m['m00'])
            dx = x - self.center[0]
            dy = y - self.center[1]
            return (True,dx,dy,0)
        else
            return (False,0,0,0)

    # Find block using corner detection (with color thresholding)
    # Returns tuple (found,x,y,t)
    def findBlock(self):
        return (False,0,0,0)

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
