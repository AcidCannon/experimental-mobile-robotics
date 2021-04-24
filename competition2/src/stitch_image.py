#! /usr/bin/python
import constants
import cv2
import move
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class StitchImage:
  def __init__(self):
    self.DEBUG = False
    self.image_raw = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
    self.bridge_object = CvBridge()
    self.image = None
    rospy.sleep(1)

  def camera_callback(self, data):
    try:
        # we select bgr8 because its the OpenCV encoding by default
        image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.image = image
        # # for test
        # if self.DEBUG:
        #     rospy.loginfo(self.identify_shape()[0])
    except CvBridgeError as e:
        rospy.loginfo(e)

  def preStitch(self):
      ANGLE = 360 / constants.STITCH_COUNT
      m = move.Move()
      for i in range(constants.STITCH_COUNT):
        m.stop()
        rospy.sleep(0.5)
        cv2.imwrite(constants.STITCH_IMAGE_COMMON_PATH_PREFIX+str(i)+".png", self.image)
        m.rotate("right", ANGLE)
      m.stop()


  def stitch(self):
    images = []
    for i in range(constants.STITCH_COUNT):
      print("=== Report === Loading " + str(i) + ".png")
      image = cv2.imread(constants.STITCH_IMAGE_COMMON_PATH_PREFIX + str(i) + ".png")
      images.append(image)
    print("=== Report === Loading complete")
    print("=== Report === Start images stiching")
    stitcher = cv2.Stitcher_create()
    status, stitched = stitcher.stitch(images)
    if status == 0:
      print("=== Report === Successfully stitched images")
      cv2.imwrite(constants.STITCH_IMAGE_COMMON_PATH_PREFIX+"pano.png", stitched)
      if self.DEBUG:
        cv2.imshow("StitchedImage", stitched)
        cv2.waitKey(0)
      return True, stitched
    else:
      statusMeaning = ['OK', 'ERR_NEED_MORE_IMGS', 'ERR_HOMOGRAPHY_EST_FAIL', 'ERR_CAMERA_PARAMS_ADJUST_FAIL']
      print("=== Report === Faild to stitch images, status code = " + str(status) + " " + statusMeaning[status])
      return False, None

if __name__ == "__main__":
  rospy.init_node("shapes_room")
  stitchImage = StitchImage()
  stitchImage.DEBUG = True
  stitchImage.preStitch()
  stitchImage.stitch()
  while not rospy.is_shutdown():
    pass