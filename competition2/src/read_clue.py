#! /usr/bin/python

import rospy
import cv2
import pytesseract
import general_purpose_OCR
import constants

class ReadClue:

    def __init__(self):
        self.generalPurposeOCR = general_purpose_OCR.GeneralPurposeOCR()

    def lobby(self):
        path = constants.CLUE_IMAGE_PATH_PREFIX + "lobby/" + "one.png"
        result = self.generalPurposeOCR.getResultFromPath(path)
        print("=== Clue ===")
        print(result)
        if "high" or "highest" in result:
            return("highest")
        else:
            return("lowest")
    
    def shapes(self):
        path = constants.CLUE_IMAGE_PATH_PREFIX + "shapes/" + "one.png"
        result = self.generalPurposeOCR.getResultFromPath(path)
        print("=== Clue ===")
        print(result)
        color = "yellow"
        shape = "cylinder"

        if "red" in result:
            color = "red"
        elif "blue" in result:
            color = "blue"
        
        if "cube" or "box" or "boxes" or "cubes" in result:
            shape = "cube"
        elif "sphere" or "shperes" in result:
            shape = "sphere"
        
        return color, shape
    
    def bandit(self):
        path = constants.CLUE_IMAGE_PATH_PREFIX + "bandit/" + "one.png"
        result = self.generalPurposeOCR.getResultFromPath(path)
        print("=== Clue ===")
        print(result)
        
        numbers = []
        for each in result.split():
            if each.isdigit():
                numbers.append(each)
        return numbers

    def maze(self):
        path = constants.CLUE_IMAGE_PATH_PREFIX + "maze/" + "one.png"
        result = self.generalPurposeOCR.getResultFromPath(path)
        print("=== Clue ===")
        print(result)
        
        numbers = []
        for each in result.split():
            if each.isdigit():
                numbers.append(each)
        return numbers
    
    def accusation(self):
        pass


if __name__ == "__main__":
    rospy.init_node("read_clue_node")
    readClue = ReadClue()
    print(readClue.lobby())