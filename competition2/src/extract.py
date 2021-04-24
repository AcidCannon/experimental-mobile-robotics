

#! /usr/bin/python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

rospy.init_node("getsign")
bridge_object = CvBridge()

def camera_callback(data):
    image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # img = cv2.cvtColor(np.copy(image), cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    min_dark = np.array([0, 0, 0])
    max_dark = np.array([57, 57, 57])
    mask = cv2.inRange(hsv, min_dark, max_dark)

    result = cv2.bitwise_and(image, image, mask = mask)
    # img = cv2.imread(result, cv2.IMREAD_GRAYSCALE)

    img = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    _,threshold = cv2.threshold(img, 0, 57, 
                            cv2.THRESH_BINARY)
    
    contours,_=cv2.findContours(threshold, cv2.RETR_TREE,
                            cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours :
        area = cv2.contourArea(cnt)
   
        if area > 400: 
            approx = cv2.approxPolyDP(cnt, 
                                    0.009 * cv2.arcLength(cnt, True), True)
    
            if(len(approx) == 4): 
                print("found sighn")
    

    cv2.imshow("result", result)
    print("boop")
    
rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback)

rospy.spin()
