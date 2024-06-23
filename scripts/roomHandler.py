from drawRoom import RoomDrawCreator, DrawRoomHorizontal, DrawRoomVertical
from room import Room

import random

class RoomHandler():
    def __init__(self) -> None:
        self.__rdc = RoomDrawCreator()
        self.__img = self.__rdc.img
        self.__draw = self.__rdc.draw
        
        self.__drh = DrawRoomHorizontal(self.__img, self.__draw)
        self.__drv = DrawRoomVertical(self.__img, self.__draw)
        
    def drawInitRoom(self, room : Room) -> None:
        """draw init room frame"""
        self.__drh.drawWall(0, room)
        self.__drh.drawWall(room.getHeight(), room)
        self.__drv.drawWall(0, room)
        self.__drv.drawWall(room.getWidth(), room)

    def splitRoom(self, room : Room) -> tuple:
        """split room randomly vertical or horizontal"""
        splitRoomRand = random.choice([self.splitRoomHorizontal, self.splitRoomVertical])
        return splitRoomRand(room)
    
    def splitRoomHorizontal(self, room : Room) -> tuple:
        split_val = self.__getRandomValueHorizontal(room)
        self.__drh.drawWall(split_val, room)
        
        door_width = self.__getRandomValue((room.getPoint()[0], room.getPoint()[0]+room.getWidth()-self.__rdc.img_params.DOOR_SIZE))
        self.__drh.drawDoor(door_width, split_val)        
        return self.getSplittedRoomHorizontal(room, split_val, door=(door_width, split_val))

    def splitRoomVertical(self, room : Room) -> tuple:
        split_val = self.__getRandomValueVertical(room)
        self.__drv.drawWall(split_val, room)
        
        door_height = self.__getRandomValue((room.getPoint()[1], room.getPoint()[1]+room.getHeight()-self.__rdc.img_params.DOOR_SIZE))
        self.__drv.drawDoor(split_val, door_height)
        return self.getSplittedRoomVertical(room, split_val, door=(split_val, door_height))
    
    def save(self) -> None:
        """Save image"""
        self.__rdc.saveImage()

    def getNewRoom(self, point, width, height, room = None) -> Room:
        """get new room"""
        return Room(point, width, height, room)

    def getSplittedRoomHorizontal(self, room : Room, val, door : tuple = None) -> tuple:
        room1 = self.getNewRoom(room.getPoint(), room.getWidth(), val)
        
        new_point = (room.getPoint()[0], room.getPoint()[1] + val)
        room2 = self.getNewRoom(new_point, room.getWidth(), room.getHeight() - val)
        
        if door:
            room1.setDoor(door)
            room2.setDoor(door)

        return room1, room2

    def getSplittedRoomVertical(self, room : Room, val, door : tuple = None) -> tuple:
        room1 = self.getNewRoom(room.getPoint(), val, room.getHeight())
        
        new_point = (room.getPoint()[0] + val, room.getPoint()[1])
        room2 = self.getNewRoom(new_point, room.getWidth()-val, room.getHeight())
        
        if door:
            room1.setDoor(door)
            room2.setDoor(door)

        return room1, room2

    def __getRandomValue(self, range : tuple) -> int:
        """Get generated random value in range (val_min, val_max)"""
        return random.randint(range[0], range[1])
    
    def __getRandomValueHorizontal(self, room : Room) -> int:
        """Get random value horizontal without doors"""
        split_area = self.__getSplitAreaHorizontal(room)
        area = random.choice(split_area)
        return self.__getRandomValue(area)
        
    def __getRandomValueVertical(self, room : Room) -> int:
        """Get random value vertical without doors"""
        split_area = self.__getSplitAreaVertical(room)
        area = random.choice(split_area)
        return self.__getRandomValue(area)  

    def __getSplitAreaVertical(self, room : Room):
        """Get vertical areas without doors"""
        min_room_size = self.__rdc.img_params.MIN_ROOM_SIZE
        door_size = self.__rdc.img_params.DOOR_SIZE
        split_area = [(room.getPoint()[0] + min_room_size, room.getPoint()[0] + room.getWidth() - min_room_size)]
        
        if split_area[0][0] < split_area[0][1]:
            for door in room.getDoor():
                if door is not None:
                    for area in split_area:
                        if door[0] in range(area[0], area[1]):
                            split_area.append((area[0], door[0]))
                            split_area.append((door[0]+door_size, area[1]))
                            split_area.remove(area)
                            break
            return split_area
        else:
            return []

    def __getSplitAreaHorizontal(self, room : Room):
        """Get horizontal areas without doors"""
        min_room_size = self.__rdc.img_params.MIN_ROOM_SIZE
        door_size = self.__rdc.img_params.DOOR_SIZE
        split_area = [(room.getPoint()[1] + min_room_size, room.getPoint()[1] + room.getHeight() - min_room_size)]
        
        if split_area[0][0] < split_area[0][1]:
            for door in room.getDoor():
                if door is not None:
                    for area in split_area:
                        if door[1] in range(area[0], area[1]):
                            split_area.append((area[0], door[1]))
                            split_area.append((door[1]+door_size, area[1]))
                            split_area.remove(area)
                            break
            return split_area
        else:
            return []