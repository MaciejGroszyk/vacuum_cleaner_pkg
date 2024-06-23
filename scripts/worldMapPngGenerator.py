
from roomHandler import RoomHandler
from roomParams import RoomParams

from room import Room

from PIL import Image, ImageDraw
import random
class WorldMapPngGenerator():
    def __init__(self) -> None:
        self.room_list = []
        self.roomParams = RoomParams()
        self.roomHandler = RoomHandler()
        
    def initRoom(self) -> None:
        init_point = (0, 0)
        width = self.roomParams.IMG_SIZE[0]
        height = self.roomParams.IMG_SIZE[1]
        room = self.__getNewRoom(init_point, width, height)
        self.roomHandler.drawInitRoom(room)
        self.room_list.append(room)

    def splitRandomRoom(self) -> None:
        random_room = self.getRandomRoom()
        r1, r2 = self.roomHandler.splitRoom(random_room)
        self.room_list.remove(random_room)
        self.room_list.append(r1)
        self.room_list.append(r2)
        
    def getRandomRoom(self) -> Room:
        return random.choice(self.room_list)

    def __getNewRoom(self, point, width, height, room = None) -> Room:
        return Room(point, width, height, room)

    def saveWorldMapPng(self):
        self.roomHandler.save()
        
    def main(self):
        self.initRoom()
        self.splitRandomRoom()
        self.splitRandomRoom()

if __name__ == "__main__":
    wmpg = WorldMapPngGenerator()
    wmpg.main()
    wmpg.saveWorldMapPng()
    