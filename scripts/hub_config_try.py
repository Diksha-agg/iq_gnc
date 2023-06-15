#!/usr/bin/env python

# Tutorial link: https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-1-display-real-time-video-feed-a98c078c708b
import rospy
import cv2

import torch
from PIL import Image as iimage
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Load PackNet model
repo = "TRI-ML/vidar"
packnet_model = torch.hub.load(repo, "PackNet", pretrained=True, trust_repo=True)
packnet_model.eval()

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.callback)

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
    except CvBridgeError as e:
      rospy.logerr(e)
    #image = cv_image
    print('cv_image',cv_image.shape)
    #image_path = image  # Replace with the path to your image file
    #image = iimage.open(image_path).convert("RGB")
    image = iimage.fromarray(cv_image.astype(np.uint8), mode="RGB")
    
    print('array_image',image.size)

    transform = transforms.Compose([transforms.Resize((256,512)),transforms.ToTensor()]) # Convert the image to a PyTorch tensor


#    transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))  # Normalize the image
    input_tensor = transform(image).unsqueeze(0)  # Add a batch dimension to the input tensor
    output = packnet_model(input_tensor)
    # Print the shape of the output
    ##print("Output shape:", (output))
    array_0 = output[0].detach().cpu().numpy()
    image_0 = array_0[0, 0, :, :]
    # Visualize the image using Matplotlib
    #plt.imshow(image_0, cmap='jet_r')
    # Apply color map
    # scaled_img = cv2.convertScaleAbs(image_0, alpha=255.0/np.amax(image_0))
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(image_0, alpha=255.0/np.amax(image_0)), cv2.COLORMAP_JET)
    # Display the image
    # cv2.imshow("Camera output depthmap", depth_colormap)
    # cv2.imshow("scaled_img", scaled_img)




    #resized_image = cv2.resize(image, (360, 640)) 

    cv2.imshow("Camera output normal", cv_image)
    #cv2.imshow("Camera output depth", image_0)
    
    # apply 'jet_r' colormap to image_0
    # image_color = cv2.applyColorMap((image_0*255).astype(np.uint8), cv2.COLORMAP_JET)
    # add colorbar scale to image
    scale = np.linspace(0, 1, 256) * 255
    scale = cv2.applyColorMap(scale.astype(np.uint8), cv2.COLORMAP_JET)
    scale = cv2.resize(scale, (50, 256))
    image_color = np.hstack((depth_colormap, scale))
    # display the image with scale
    cv2.imshow('Camera output depth', image_color)

    cv2.waitKey(3)

def main():
	
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
