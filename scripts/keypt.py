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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError
from iq_gnc.py_gnc_functions import *
# from matplotlib import pyplot as plt
from math import radians, sin, cos
from collections import deque



class camera_1:

  def __init__(self, gnc:gnc_api):
    self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.callback)
    # self.position_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.position_callback)    
    self.velocity_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velocity_callback)
    self.position_x = deque(maxlen=20)
    self.position_y = deque(maxlen=20)
    self.velocity_trans_x =0
    self.velocity_trans_y =0
    self.vx = 0
    self.vy = 0
    self.drone = gnc
    self.velocity_x_deque = deque(maxlen=20)
    self.velocity_y_deque = deque(maxlen=20)
    self.buffered_frames = deque(maxlen=20)

    # self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)
    self.tic = time.time()

    self.image_num = 0

  # def position_callback(self, data:Odometry):
  #       # Access the position value from the received message
  #       self.position_x = data.pose.pose.position.x
  #       self.position_y = data.pose.pose.position.y

  def velocity_callback(self, data:TwistStamped):
        # Access the velocity value from the received message
        self.local_offset_g = 0
        
        self.vx = data.twist.linear.x
        self.vy = data.twist.linear.y
        self.velocity_trans_x = self.vx * cos(radians((self.local_offset_g - 90))) - self.vy * sin(
            radians((self.local_offset_g - 90))
        )

        self.velocity_trans_y = self.vx * sin(radians((self.local_offset_g - 90))) + self.vy * cos(
            radians((self.local_offset_g - 90))
        )
    
  def callback(self,data:Image):
    
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      
    hsv = cv_image #cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
    # Threshold of blue in HSV space
    lower_blue = np.array([60, 0 ,0])
    upper_blue = np.array([255, 0, 0])

    cv_img2 = cv_image

    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(cv_image, cv_image, mask = mask)
    
    	
    image = cv_image
    resized_image = cv2.resize(image, (1024, 1024)) 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # pos_x = self.position_x
    # pos_y = self.position_y
    # print('posx:',pos_x)
    
    # print('vel_y',self.velocity_trans_y)
       
    # cv2.imshow('gray_img', gray)
    
    if(self.drone.get_current_location().z>0.1):
      self.image_num+=1
      if len(self.buffered_frames)==self.buffered_frames.maxlen:
        
        print('vel_x',self.velocity_x_deque[-1])
        print('pos_x',self.position_x[-1])
        toc = time.time()
        time_diff = toc - self.tic
        self.tic = time.time()
        if(self.velocity_x_deque[-1]!=self.velocity_x_deque[-2]):
          print('deque vel',self.velocity_x_deque[-1])
          print('timediff: ',time_diff)
          # cv2.imwrite(str(self.image_num) +'-' + str("{:.4f}".format(self.position_x[-1]))+'x'+'-' + str("{:.4f}".format(self.position_y[-1]))+'y'+'-' + str("{:.4f}".format(self.velocity_x_deque[-1]))+'vx' +'-' + str("{:.4f}".format(self.velocity_y_deque[-1]))+'vy'+'-' + str("{:.4f}".format(time_diff))+'t'+ 'R.jpg', self.buffered_frames[-1])
          # cv2.imwrite(str(self.image_num) +'-' + str("{:.4f}".format(self.position_x[-2]))+'x'+'-' + str("{:.4f}".format(self.position_y[-2]))+'y'+'-' + str("{:.4f}".format(self.velocity_x_deque[-2]))+'vx' +'-' + str("{:.4f}".format(self.velocity_y_deque[-2]))+'vy'+'-' + str("{:.4f}".format(time_diff))+'t'+ 'L.jpg', self.buffered_frames[-2])
           
    
    self.buffered_frames.append(gray)
    self.position_x.append(self.drone.get_current_location().x)
    self.position_y.append(self.drone.get_current_location().y)
    imgR = self.buffered_frames[-1]
    imgL = self.buffered_frames[-2]
    self.velocity_x_deque.append(self.velocity_trans_x)
    self.velocity_y_deque.append(self.velocity_trans_y)
    

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

    # cv2.imshow('disparity_map',disparity_map)
    # nonzero_coords = np.argwhere(disparity_map != 0)
    # x_vals = nonzero_coords[:, 1]
    # y_vals = nonzero_coords[:, 0]


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
    # rate.sleep()
    cv2.waitKey(10)

def main():
  drone1 = gnc_api()
  camera_1(drone1)
  
  try:
    #frame = 0
    rospy.spin()
    #frame+=1
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    # rate = rospy.Rate(3)
    
    main()



