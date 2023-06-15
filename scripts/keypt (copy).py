#!/usr/bin/env python

# Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
import rospy
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import math
import time
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
    self.tic = time.time()

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
    
    	
    image = cv_image
    resized_image = cv2.resize(image, (1024, 1024)) 
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

   
    cv2.imshow('gray_img', gray)
    
    if(drone1.get_current_location().z>0.1):
      self.i+=1
      if len(buffered_frames)>20:
        buffered_frames.pop(0)
        positionx.pop(0)
        positiony.pop(0)
        # print('write',self.i)
        toc = time.time()
        time_diff = toc - self.tic
        self.tic = time.time()
        # print('timediff: ',time_diff)
        # cv2.imwrite(str(self.i) +'-' + str("{:.4f}".format(positionx[-1]))+'x' +'-' + str("{:.4f}".format(positiony[-1]))+'y'+ 'L.jpg', buffered_frames[-1])
        # cv2.imwrite(str(self.i) +'-' + str("{:.4f}".format(positionx[-2]))+'x' +'-' + str("{:.4f}".format(positiony[-2]))+'y'+ 'R.jpg', buffered_frames[-2])
        # print(positionx[-1])
          
    
    buffered_frames.append(gray)
    positionx.append(drone1.get_current_location().x)
    positiony.append(drone1.get_current_location().y)
    print(drone1.get_current_location().x)
    imgL = buffered_frames[-1]
    imgR = buffered_frames[-2]
    

    orb = cv2.ORB_create()

    RKeypoints, RDescriptors = orb.detectAndCompute(imgR,None)
    LKeypoints, LDescriptors = orb.detectAndCompute(imgL,None)
 

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(RDescriptors,LDescriptors)
    matches = sorted(matches, key = lambda x:x.distance)
    points1 = np.zeros((len(matches),2),dtype=np.float32)
    points2 = np.zeros((len(matches),2),dtype=np.float32)

    for c,value in enumerate(matches):
        points1[c,:] = RKeypoints[value.queryIdx].pt
        points2[c,:] = LKeypoints[value.trainIdx].pt
 

    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)

    disparity_map = np.zeros_like(imgL, dtype=np.float32)

    for i,value in enumerate(matches):
        if(mask[i]==1):
            SADx = abs(RKeypoints[value.queryIdx].pt[0]-LKeypoints[value.trainIdx].pt[0])
            SADy = abs(RKeypoints[value.queryIdx].pt[1]-LKeypoints[value.trainIdx].pt[1])
            disparity_value = max(SADx, SADy)
            # print(disparity_value)
            x, y = np.int32(LKeypoints[value.trainIdx].pt)
            disparity_map[y, x] = disparity_value

    cv2.imshow('disparity_map',disparity_map)
    nonzero_coords = np.argwhere(disparity_map != 0)
    x_vals = nonzero_coords[:, 1]
    y_vals = nonzero_coords[:, 0]


    # plt.scatter(x_vals, y_vals)
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.title('Non-zero Values in Disparity Map')
    # plt.show()

    # image = np.zeros((1024, 1024, 3), dtype=np.uint8)

    # # Generate some random data points
    # np.random.seed(0)
    # data = np.random.randint(0, 500, size=(100, 2))

    # # Plot the data points
    # for point in data:
    #     x, y = point
    #     cv2.circle(image, (x, y), 3, (0, 255, 0), -1)

    # Display the image
    # cv2.imshow('Scatter Plot', image)
        
    cv2.waitKey(1)

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
    

