from PIL import Image, ImageDraw

import os


class ImageParams():
    def __init__(self):
        self.IMG_SIZE = (480,640)
        self.WALL_SIZE = 10
        self.DOOR_SIZE = 40
        self.EMPTY_COLOR_VALUE = 0
        self.WALL_COLOR_VALUE = 255
        
class WorldMapPngGenerator():

    def __init__(self):
        self.__current_file_path = os.path.abspath(os.path.dirname(__file__))
        
        self.img_params = ImageParams()
        self.img = self.getEmptyImg()
        self.draw = ImageDraw.Draw(self.img)
        
        self.initImage()
        
    def initImage(self):
        self.generateImgFrame()

    def generateImgFrame(self):
        w = self.img_params.IMG_SIZE[0]
        h =  self.img_params.IMG_SIZE[1]
        
        wall_color = self.getWallColorImage()
        self.draw.line((0, 0, 0, h), fill=wall_color, width= self.img_params.WALL_SIZE)
        self.draw.line((w, 0, w, h), fill=wall_color, width= self.img_params.WALL_SIZE)
        self.draw.line((0, 0, w, 0), fill=wall_color, width= self.img_params.WALL_SIZE)
        self.draw.line((0, h, w, h), fill=wall_color, width= self.img_params.WALL_SIZE)
    
    def getEmptyColorImage(self) -> tuple:
        ecv = self.img_params.EMPTY_COLOR_VALUE
        return (ecv, ecv, ecv)
    
    def getWallColorImage(self) -> tuple:
        wcv = self.img_params.WALL_COLOR_VALUE
        return (wcv, wcv, wcv)
    
    def getEmptyImg(self) -> Image:
        empty_color = self.getEmptyColorImage()
        return Image.new('RGB', self.img_params.IMG_SIZE, empty_color)

    def saveImage(self):
        IMG_NAME = "/test.png"
        self.img.save(self.__current_file_path+IMG_NAME)

if __name__ == "__main__":
    wmpg = WorldMapPngGenerator()
    wmpg.initImage()
    wmpg.saveImage()

    