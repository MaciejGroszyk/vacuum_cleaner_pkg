from PIL import Image, ImageDraw

import os
import random

class ImageParams():
    def __init__(self):
        self.IMG_SIZE = (480,640)
        self.WALL_SIZE = 10
        self.DOOR_SIZE = 40
        self.EMPTY_COLOR_VALUE = 0
        self.WALL_COLOR_VALUE = 255

class ImgDrawCreator():
    def __init__(self):
        self.img_params = ImageParams()
        self.__current_file_path = os.path.abspath(os.path.dirname(__file__)) 
        
        self.img = self.getEmptyImg()
        self.draw = self.getDrawInstance()
        

    def getEmptyImg(self) -> Image:
        ecv = self.img_params.EMPTY_COLOR_VALUE
        empty_color = (ecv, ecv, ecv)
        return Image.new('RGB', self.img_params.IMG_SIZE, empty_color)

    def getDrawInstance(self) -> ImageDraw:
        return ImageDraw.Draw(self.img)

    def saveImage(self):
        IMG_NAME = "/test2.png"
        self.img.save(self.__current_file_path+IMG_NAME)

class DrawImage():
    def __init__(self, img, draw): 
        self.img_params = ImageParams()
        self.img = img 
        self.draw = draw

    def drawLine(self):
        pass
    
    def drawRandomLine(self):
        pass

    def drawRandomLineWithDoor(self):
        pass

    def drawDoor(self):
        pass

    def getEmptyColorImage(self) -> tuple:
        ecv = self.img_params.EMPTY_COLOR_VALUE
        return (ecv, ecv, ecv)

    def getWallColorImage(self) -> tuple:
        wcv = self.img_params.WALL_COLOR_VALUE
        return (wcv, wcv, wcv)
    
    def getRandomValue(self, val) -> int:
        return random.randint(0, val)

    def getRandomWidthHeight(self) -> tuple:
        w = self.getRandomValue(self.img_params.IMG_SIZE[0])
        h = self.getRandomValue(self.img_params.IMG_SIZE[1])
        return w, h

class DrawImageHorizontal(DrawImage):
    def drawLine(self, height):
        w = self.img_params.IMG_SIZE[0]
        self.draw.line((0, height, w, height), fill=self.getWallColorImage(), width= self.img_params.WALL_SIZE)

    def drawDoor(self, height, door_pose):
        self.draw.line((door_pose, height, door_pose + self.img_params.DOOR_SIZE , height),
                        fill=self.getEmptyColorImage(), width= self.img_params.WALL_SIZE)

    def drawRandomLine(self):
        h = self.img_params.IMG_SIZE[1]
        self.drawLine(self.getRandomValue(h))

    def drawRandomLineWithDoor(self):
        w, h = self.getRandomWidthHeight()
        self.drawLine(h)
        self.drawDoor(h, w)

class DrawImageVertical(DrawImage):
    def drawLine(self, width):
        h =  self.img_params.IMG_SIZE[1]
        self.draw.line((width, 0, width, h), fill=self.getWallColorImage(), width= self.img_params.WALL_SIZE)

    def drawDoor(self, width, door_pose):
        self.draw.line((width, door_pose, width, door_pose + self.img_params.DOOR_SIZE ),
                        fill=self.getEmptyColorImage(), width= self.img_params.WALL_SIZE)

    def drawRandomLine(self):
        w = self.img_params.IMG_SIZE[0]
        self.drawLine(self.getRandomValue(w))

    def drawRandomLineWithDoor(self):
        w, h = self.getRandomWidthHeight()
        self.drawLine(w)
        self.drawDoor(w, h)