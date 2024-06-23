from PIL import Image, ImageDraw
from room import Room
from roomParams import RoomParams
import os

class RoomDrawCreator():
    def __init__(self):
        self.img_params = RoomParams()
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

class DrawRoom():
    def __init__(self, img, draw): 
        self.img_params = RoomParams()
        self.img = img 
        self.draw = draw

    def drawWall(self):
        pass

    def drawDoor(self):
        pass

    def getEmptyColor(self) -> tuple:
        ecv = self.img_params.EMPTY_COLOR_VALUE
        return (ecv, ecv, ecv)

    def getWallColor(self) -> tuple:
        wcv = self.img_params.WALL_COLOR_VALUE
        return (wcv, wcv, wcv)

class DrawRoomHorizontal(DrawRoom):
    def drawWall(self, height, room : Room):
        w_start = room.getPoint()[0]
        w_end = room.getPoint()[0] + room.getWidth()
        self.draw.line((w_start, height, w_end, height), fill=self.getWallColor(), width= self.img_params.WALL_SIZE)

    def drawDoor(self, width, height):
        self.draw.line((width, height, width + self.img_params.DOOR_SIZE , height),
                        fill=self.getEmptyColor(), width= self.img_params.WALL_SIZE)

class DrawRoomVertical(DrawRoom):
    def drawWall(self, width, room : Room):
        h_start = room.getPoint()[1]
        h_end =  room.getPoint()[1] +room.getHeight()
        self.draw.line((width, h_start, width, h_end), fill=self.getWallColor(), width= self.img_params.WALL_SIZE)

    def drawDoor(self, width, height):
        self.draw.line((width,height, width, height + self.img_params.DOOR_SIZE ),
                        fill=self.getEmptyColor(), width= self.img_params.WALL_SIZE)
