#! /usr/bin/python
import constants
import cv2

class StitchImage:
  def __init__(self):
    self.DEBUG = False

  def stitch(self):
    images = []
    for i in range(constants.STITCH_COUNT):
      print("=== Report === Loading " + str(i) + ".jpg")
      image = cv2.imread(constants.STITCH_IMAGE_COMMON_PATH_PREFIX + str(i) + ".jpg")
      images.append(image)
    print("=== Report === Loading complete")
    print("=== Report === Start images stiching")
    stitcher = cv2.Stitcher_create()
    status, stitched = stitcher.stitch(images)
    if status == 0:
      print("=== Report === Successfully stitched images")
      if self.DEBUG:
        cv2.imshow("StitchedImage", stitched)
        cv2.waitKey(0)
    else:
        statusMeaning = ['OK', 'ERR_NEED_MORE_IMGS', 'ERR_HOMOGRAPHY_EST_FAIL', 'ERR_CAMERA_PARAMS_ADJUST_FAIL']
        print("=== Report === Faild to stitch images, status code = " + status + " " + statusMeaning[status])

if __name__ == "__main__":
  stitchImage = StitchImage()
  stitchImage.DEBUG = True
  stitchImage.stitch()