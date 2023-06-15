# #!/usr/bin/env python

# # Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
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

def main():
    # rospy.init_node('camera_read', anonymous=False)
    
    drone1 = gnc_api()
    # rate = rospy.Rate(3)
    # image_sub = rospy.Subscriber("/webcam/image_raw", Image)
    # bridge = CvBridge()
    
    # try:
    #   cv_image = bridge.imgmsg_to_cv2(image_sub, desired_encoding="bgr8")
    # except CvBridgeError as e:
    #   rospy.logerr(e)
    
    print(drone1.get_current_location().x)
    

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()



#!/usr/bin/env python

# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from iq_gnc.py_gnc_functions import *

# def callback(data):
#     bridge = CvBridge()

#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
#     except CvBridgeError as e:
#         rospy.logerr(e)

#     drone1 = gnc_api()
#     print(drone1.get_current_location().x)

# def main():
#     rospy.init_node('camera_read', anonymous=False)
#     image_sub = rospy.Subscriber("/webcam/image_raw", Image, callback)
#     rate = rospy.Rate(3)  # Specify your desired rate in Hz
    
#     while not rospy.is_shutdown():
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         exit()
