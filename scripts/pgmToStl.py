from worldMapModelGenerator import WorldMapModelGenerator
import os

import cv2
import numpy as np

class PgmToStl():
    def __init__(self) -> None:
        self.__current_file_path   = os.path.abspath(os.path.dirname(__file__))
        self.__package_path        = os.path.dirname(self.__current_file_path)

        self.PGM_FILE = "/data/p109/lab_map.pgm"
        self.PNG_FILE = "/data/p109/lab_map.png"
        
        self.wmmg = WorldMapModelGenerator()
        self.image = None

    def readPGMFile(self):
        self.image = cv2.imread(self.__package_path + self.PGM_FILE)

    def savePgmAsPng(self):
        cv2.imwrite(self.__package_path + self.PNG_FILE, self.image)
        
    def deleteNoiseFromPgm(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated = cv2.dilate(self.image, kernel)
        
        blured = cv2.medianBlur(dilated, 5)
        img = self.imgToBlackColors(blured)
        return img

    def imgToBlackColors(self, img_in):
        b, g, r = cv2.split(img_in)
        b[b < 250] = 0
        g[g < 250] = 0
        r[r < 250] = 0
        return cv2.merge((b,g,r))

    def main(self):
        self.readPGMFile()
        self.savePgmAsPng()

        img = self.deleteNoiseFromPgm()
        self.wmmg.LOCAL_MODEL_PATH   = "/data/p109"
        self.wmmg.CURRENT_MODEL_PNG  = "/data/p109/lab_map.png"
        self.wmmg.MODEL_TIFF_FOLDER  = "/data/p109/tiff_files"
        self.wmmg.worldFromPngModelGenerator(img)
        cv2.imwrite(self.__package_path + self.PNG_FILE, img)

if __name__ == "__main__":
    pts = PgmToStl()
    pts.main()
