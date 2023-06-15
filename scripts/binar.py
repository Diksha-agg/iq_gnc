#!/usr/bin/env python

# Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from iq_gnc.py_gnc_functions import *
# from matplotlib import pyplot as plt


buffered_frames = []
positionx = []
positiony = []

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.callback)
    # self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)

    self.i = 0
    
  def callback(self,data):
    
    bridge = CvBridge()
    frame =0
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      
    hsv = cv_image #cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
    # Threshold of blue in HSV space
    lower_blue = np.array([60, 0 ,0])#np.array([60, 35, 140])
    upper_blue = np.array([255, 0, 0])#np.array([180, 255, 255])

    cv_img2 = cv_image

    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    
    	
   # print(cv_image.shape[0])
    image = cv_image
     #frame = 0
    resized_image = cv2.resize(image, (1024, 1024)) 
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

   
    #while True:
      #cv2.imshow("Camera output normal", image)
    # cv2.imshow("Camera output resized", resized_image)
    #cv2.imshow("Camera img2 resized", cv_img2)
    #cv2.imshow('mask', mask)
    # cv2.imshow('result', result)
    cv2.imshow('gray_img', gray)
    
    # print(drone1.get_current_location().z)
    
    if(drone1.get_current_location().z>0.1):
      self.i+=1
      if len(buffered_frames)>20:
        buffered_frames.pop(0)
        # positionx.pop(0)
        # positiony.pop(0)
        print('write',self.i)
        cv2.imwrite(str(self.i) + 'L.jpg', buffered_frames[-1])
        cv2.imwrite(str(self.i) + 'R.jpg', buffered_frames[-2])
          
    buffered_frames.append(gray)
    # positionx.append(drone1.get_current_location().x)
    # positiony.append(drone1.get_current_location().y)
    # print(buffered_frames[0].shape)
    # stereo = cv2.StereoBM_create(numDisparities=16*20, blockSize=15)
    # disparity = stereo.compute(buffered_frames[0],buffered_frames[-1])
    # print('distance: ')
    # print(math.sqrt((positionx[-1]-positionx[-0])**2+(positiony[-1]-positiony[-0])**2))
    # print('distancexy: ')
    # print(positionx[-1],positionx[-0])
    # print(math.sqrt(2**2))
    # print(buffered_frames.len)
    # plt.imshow(disparity,'gray')
    # plt.show()
    # print(len(buffered_frames))
    # stereo = cv2.StereoSGBM_create(minDisparity = 0, numDisparities = 16, 
    #                                blockSize = 9, uniquenessRatio = 50, 
    #                                speckleRange = 1, speckleWindowSize = 190,
    #                                disp12MaxDiff = 0,
    # P1 = 91,
    # P2 = 289
    # )
    stereo = cv2.StereoSGBM_create(numDisparities = 16, 
                                   blockSize = 9)
    left_disp = stereo.compute(buffered_frames[0],buffered_frames[-1])
    norm_image = cv2.normalize(left_disp, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX)
    # cv2.imshow("SGBM", norm_image)
    # cv2.imshow('disparity', left_disp)
    # cv2.imshow('buf_img', buffered_frames)
    
    cv2.waitKey(3)

def main():
  
  camera_1()
  
  try:
    #frame = 0
    rospy.spin()
    #frame+=1
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    
    drone1 = gnc_api()
    main()
    

