
class Room():
    """Room description"""
    def __init__(self, point : tuple,  width : int, height : int, door : tuple = None) -> None:
        self.__point = point
        self.__height = height
        self.__width = width
        self.__door = [door]

    def getPoint(self) -> tuple:
        """get room init pose (width, height)"""
        return self.__point

    def getHeight(self) -> int:
        """get room height"""
        return self.__height

    def getWidth(self) -> int:
        """get room width"""
        return self.__width
    
    def getDoor(self):
        """get room door pose (width, height)"""
        return self.__door
    
    def setDoor(self, door : tuple) -> None:
        """get room room pose"""
        self.__door.append(door)
    
    def isDoor(self) -> bool:
        """check if room has door"""
        return (self.__door is None)
