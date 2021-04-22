import cv2
import pytesseract
import constants

class GeneralPurposeOCR:

  def getResultFromPath(self, path):
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    result = pytesseract.image_to_string(image, config='--psm 6 --oem 3').strip()
    return result


  def getResultFromImage(self, image):
    result = pytesseract.image_to_string(image, config='--psm 6 --oem 3').strip()
    return result

if __name__ == "__main__":
  generalPurposeOCR = GeneralPurposeOCR()
  print(generalPurposeOCR.getResultFromPath(constants.MAP_NUMBER_ROOM))