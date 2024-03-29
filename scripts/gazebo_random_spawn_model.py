" Spawn model to gazebo "
import random
import time
import os
import rclpy
from gazebo_msgs.srv import SpawnEntity
import cv2

class GazeboRandomSpawnModel():
    """Spawning models to gazebo"""
    def __init__(self):
        self.__image = None
        self.__current_file_path = os.path.abspath(os.path.dirname(__file__))
        self.__package_path = os.path.dirname(self.__current_file_path)
        self.CURRENT_MODEL_PNG = "/data/map_model/current_map_img.png"
        self.CURRENT_MAP_SDF = "/data/map_model/model.sdf"
        # config

    def readModelSdf(self) -> str:
        """Read current map model sdf"""
        with open(self.__package_path + self.CURRENT_MAP_SDF, encoding='UTF-8') as map_sdf:
            return map_sdf.read()

    def spawnMap(self):
        """Spawn model map to gazebo"""
        xml = self.readModelSdf()
        self.requestSpawn(xml)

    def getImageParams(self):
        """Read image and get params"""
        self.__image = cv2.imread(self.__package_path + self.CURRENT_MODEL_PNG)
        height, width, _ = self.__image.shape
        return height, width

    def isWall(self, pose) -> bool:
        """Check if img pose is wall"""
        WALL_VALUE = 255
        return all(p == WALL_VALUE for p in pose)


    def generateRandomPoseFromImg(self, iter_max: int = 50) -> tuple:
        """Generate random pose in range current img map"""
        height, width = self.getImageParams()
        random_pose = []
        i = 0
        while self.isWall(random_pose) and i < iter_max:
            rand_h = random.randint(1, height)
            rand_w = random.randint(1, width)
            random_pose = self.__image[rand_h][rand_w]
            i += 1

        if i >= iter_max:
            return (0, 0)
        return (rand_h, rand_w)

    def getRandomPose(self) -> str:
        """Get random pose in img range"""
        new_h, new_w = self.generateRandomPoseFromImg()
        SCALE = 200
        return f"<pose> {new_w/SCALE:.1f} {new_h/SCALE:.1f} 0 0 0</pose>"

    def spawnModelOnce(self):
        """Spawn model once"""
        xml = self.generateXmlSphere()
        self.requestSpawn(xml)

    def spawnModelWithPeriod(self, period_sec: float = 2.0, count: int = 10):
        """Spawn model with period in seconds"""
        i = 0
        while i < count:
            self.spawnModelOnce()
            time.sleep(period_sec)
            i += 1

    def generateXmlSphereWithPose(self, pose) -> str:
        """Generate xml sphere with set pose"""
        radius = str(0.5)
        return "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"random_obstacle\">"+pose+"<static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>"+radius+"</radius></sphere></geometry></visual><collision name=\"collision\"><geometry><sphere><radius>"+radius+"</radius></sphere></geometry></collision></link></model></sdf>"

    def generateXmlSphere(self) -> str:
        """Generate xml sphere on random pose"""
        radius = str(0.5)
        return "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"random_obstacle\">"+self.getRandomPose()+"<static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>"+radius+"</radius></sphere></geometry></visual><collision name=\"collision\"><geometry><sphere><radius>"+radius+"</radius></sphere></geometry></collision></link></model></sdf>"

    @staticmethod
    def requestSpawn(xml: str):
        """Spawn gazebo model with node"""
        rclpy.init()
        node = rclpy.create_node('spawn_entity')
        client = node.create_client(SpawnEntity, 'spawn_entity')
        if not client.service_is_ready():
            client.wait_for_service()
        request = SpawnEntity.Request()
        request.xml = xml
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print(f"response: {future.result()}")
        else:
            raise RuntimeError(f"exception while calling service: {future.exception()}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    GRSM = GazeboRandomSpawnModel()
    # GRSM.spawnModelOnce()
    print(GRSM.getImageParams())
    # print(GRSM.generateRandomPoseFromImg())
    # print(GRSM.getRandomPose())

    # GRSM.spawnModelWithPeriod(count=2)
    GRSM.spawnMap()
    # GRSM.spawnCorners()

# if len(sys.argv) < 2:
#     print('usage: ros2 run my_package my_node.py -- example.urdf')
#     sys.exit(1)

# f = open(sys.argv[1], 'r')
# requestSpawn(f.read())
