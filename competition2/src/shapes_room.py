#! /usr/bin/python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ShapesRoom:

  def __init__(self):
    self.DEBUG = False
    self.image_raw = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
    self.bridge_object = CvBridge()
    self.image = None
    self.count = 0
    rospy.sleep(1)

  def getResult(self):
    return self.count

  def camera_callback(self, data):
    try:
        # we select bgr8 because its the OpenCV encoding by default
        image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.image = image
        # for test
        if self.DEBUG:
            rospy.loginfo(self.identify_shape()[0])
    except CvBridgeError as e:
        rospy.loginfo(e)


  def identify_shape(self, color, shape):
    '''
        sites to find colour: 
        https://imagecolorpicker.com/
        https://www.rgbtohex.net/hextorgb/
        https://www.rapidtables.com/web/color/RGB_Color.html
    '''

    img = cv2.cvtColor(np.copy(self.image), cv2.COLOR_BGR2RGB)
    # colour picker rgb(102,0,0)
    # min_red = np.array([100, 17, 15])
    # max_red = np.array([255, 50, 56])
    min_red = np.array([100, 0, 0])
    max_red = np.array([255, 0, 0])
    # colour picker rgb(0,0,102)
    min_blue = np.array([0, 0, 100])
    max_blue = np.array([0, 0, 255])

    # colour picker rgb(122,122,0)
    min_yellow = np.array([100, 100, 0])
    max_yellow = np.array([255, 255, 0])

    red_mask = cv2.inRange(img, min_red, max_red)
    blue_mask = cv2.inRange(img, min_blue, max_blue)
    yellow_mask = cv2.inRange(img, min_yellow, max_yellow)

    target_mask = red_mask
    if color == 'red':
      pass
    elif color == 'yellow':
      target_mask = yellow_mask
    else:
      target_mask = blue_mask
    print("=== Report === Target color = " + color)    
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    result = cv2.bitwise_and(img, img, mask=target_mask)
    
    _, result = cv2.threshold(result, 50, 255, cv2.THRESH_BINARY_INV)
    result = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    _, result = cv2.threshold(result, 254, 255, cv2.THRESH_BINARY_INV)
    
    # cv2.imshow("result", result)
    # cv2.waitKey(0)
    print("=== Report === Target shape = " + shape)
    target_shape = 'cube'
    if shape == 'cude':
      pass
    elif shape == 'sphere':
      target_shape = 'sphere'
    else:
      target_shape = 'cylinder'

    # detector = cv2.SimpleBlobDetector()
    # keypoints = detector.detect(result)
    # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    contours, _ = cv2.findContours(result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
        print("=== Report === Found", color, shape, "Polygonal vertices length =", len(approx))
        cv2.drawContours(img, [approx], 0, (255,0,0), 5)
    
    self.count += len(contours)
    print("=== Report ===", "Current total count =", self.count)

    cv2.imshow("result", img)
    cv2.waitKey(0)

    def close(self):
        self.image_raw.unregister()

if __name__ == "__main__":
    rospy.init_node("shapes_room")
    shapesRoom = ShapesRoom()
    shapesRoom.identify_shape('red', 'cube')
    while not rospy.is_shutdown():
        pass
    