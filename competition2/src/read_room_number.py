#! /usr/bin/python
import cv2
import pytesseract
import rospy
import constants

class ReadRoomNumber:
    def __init__(self):
        self.readPath = constants.MAP_NUMBER_ROOM
        self.writePath = constants.NUMBERED_LOCATIONS

    def writeYaml(self):
        if not self.result:
            pass
        else:
            keys = list(self.result.keys())
            keys.pop(-1)
            with open(self.writePath, 'a') as f:
                f.write("0: lobby\n")
                for key in keys:
                    f.write(self.result[key]+": "+key+'\n')
        print("=== Report === Write room number pair to yaml.")

    def readRoomNumber(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        _, img = cv2.threshold(img, 230, 255, cv2.THRESH_BINARY_INV)
        result = {}
        for i in range(ord('a'), ord('q')+1):
            result[chr(i)] = -1
        result['LOBBY'] = -1
        # 'x': [(col_beginning, row_beginning), (col_end, row_end)]
        crop_coord = {
            'a': [(26,26), (132,128)],
            'b': [(23,184), (129,362)],
            'c': [(51,411), (130, 559)],
            'd': [(21,609), (127,813)],
            'e': [(177,57), (228, 126)],
            'f': [(178,178), (230,228)],
            'g': [(170,280), (390,392)],
            'h': [(178,609), (352,766)],
            'i': [(266, 28), (384,126)],
            'j': [(238,178), (389,226)],
            'k': [(429,69), (548,269)],
            'l': [(428,326), (551,443)],
            'm': [(433,486), (552,553)],
            'n': [(394,603), (541,762)],
            'o': [(594,68), (752,167)],
            'p': [(595,224), (757,550)],
            'q': [(590,602), (760,758)],
            'LOBBY':[(166,404), (276,555)]
        }
        print("=== Report === Read room number map from png file.")
        for key in crop_coord.keys():
            col_beginning = crop_coord[key][0][0]
            row_beginning = crop_coord[key][0][1]
            col_end = crop_coord[key][1][0]
            row_end = crop_coord[key][1][1]
            cropped = img[row_beginning:row_end+1, col_beginning:col_end+1]
            x = pytesseract.image_to_string(cropped, config='--psm 6 --oem 3 -c tessedit_char_whitelist=0123456789LOBBY').strip()
            print("=== OCR result ===", x)
            # rospy.loginfo("=== OCR result ===", x)
            # cv2.imshow("img", cropped)
            # cv2.waitKey(0)
            result[key] = x
            self.result = result
        return result

if __name__ == "__main__":
  readRoomNumber = ReadRoomNumber()
  result = readRoomNumber.readRoomNumber(readRoomNumber.readPath)
  print(result)
  readRoomNumber.writeYaml()