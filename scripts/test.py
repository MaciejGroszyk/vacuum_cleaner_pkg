import os
import trimesh
import matplotlib.pyplot as plt
import numpy as np
from skimage.draw import random_shapes, disk
from skimage.util import invert
from pcg_gazebo.generators import HeightmapGenerator
from pcg_gazebo.generators.biomes import Biome, WhittakerBiome


import cv2
from PIL import Image

import numpy as np
import PIL as pillow
import matplotlib.pyplot as plt


class MapGenerator():
    def __init__(self) -> None:
    
        self.__image_folder_path    = 'random_maps'
        self.__input_image_name     = 'random_map.png'
        self.__new_image_name       = 'new_random_map.png'


        self.__current_file_path   = os.path.abspath(os.path.dirname(__file__))
        self.__package_path        = os.path.dirname(self.__current_file_path)


        self.__imgage = None
        self.__scene = None

    def getPath(self, file_name):
        return os.path.join(self.__package_path, self.__image_folder_path, file_name)
    
    def getImage(self, image_name):
        self.__imgage = Image.open(self.getPath(image_name))

    def getSelfImage(self):
        self.__imgage = Image.open(self.getPath(self.__input_image_name))

    def normalizeImage(self):
        self.__imgage = self.__imgage.resize(65, 65)
        self.__imgage = (self.__imgage- np.min(self.__imgage)) * (255.0 / (np.max(self.__imgage) - np.min(self.__imgage)))

    def generateScene(self):
        hg = HeightmapGenerator(map_size=[10, 10, 2])
        hg.add_layer_from_file(self.getPath('random_map copy.png'))
        self.__scene = trimesh.Scene([hg.heightmap.mesh.mesh])

    def saveSceneAsStl(self, stl_name):
        self.__scene.export(self.getPath(stl_name))


    def showScene(self):
        self.__scene.show()



if __name__ == '__main__':
    MG = MapGenerator()
    MG.getSelfImage()
    MG.generateScene()
    MG.showScene()
    # MG.saveSceneAsStl('stuff1.stl')


# image_path = os.path.join(package_path  , image_folder_path, input_image_name)
# print(image_path)

# final_image = Image.open(image_path)
# final_image = final_image.resize((65, 65))
# # Normalize the image with values between 0 and 255
# normalized_image = (final_image - np.min(final_image)) * (255.0 / (np.max(final_image) - np.min(final_image)))
# normalized_image = normalized_image.astype('uint8')

# # Convert the normalized numpy array to an image using Pillow
# image = pillow.Image.fromarray(normalized_image)

# image.save('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map1.png')
# image = Image.open('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map.png')
# # Create a heightmap generator
# hg = HeightmapGenerator(
#     map_size=[10, 10, 2])
# # hg.add_perlin_noise_layer(freq=110.0, octaves=10)
# hg.add_layer_from_file("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map copy.png")

# scene = trimesh.Scene([hg.heightmap.mesh.mesh])
# # scene.show()


# scene.export('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/stuff.stl')



