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



from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.visualization import plot_workspace, plot_occupancy_grid
from pcg_gazebo.generators.creators import box_factory
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.simulation import SimulationModel




class MapGenerator(HeightmapGenerator):
    def __init__(self) -> None:
    
        self.__image_folder_path    = 'random_maps'
        self.__input_image_name     = 'random_map.png'
        self.__new_image_name       = 'new_random_map.png'


        self.__current_file_path   = os.path.abspath(os.path.dirname(__file__))
        self.__package_path        = os.path.dirname(self.__current_file_path)


        self.__image = None
        self.__scene = None

    def getPath(self, file_name):
        return os.path.join(self.__package_path, self.__image_folder_path, file_name)
    
    def getImage(self, image_name):
        self.__image = Image.open(self.getPath(image_name))

        self.__image = np.asarray(self.__image)

    def getSelfImage(self):
        self.__image = Image.open(self.getPath(self.__input_image_name))
        self.__image = np.asarray(self.__image)

    def normalizeImage(self):
        self.__image = self.__image.resize(65, 65)
        self.__image = (self.__image- np.min(self.__image)) * (255.0 / (np.max(self.__image) - np.min(self.__image)))
        self.__image = self.__image.astype('uint8')

    def generateScene(self):
        hg = HeightmapGenerator(map_size=[10, 10, 5])

        # hg.add_layer_from_file(self.getPath('random_map copy1.png'))
        hg.add_custom_layer(self.__image)
        self.__scene = trimesh.Scene([hg.heightmap.mesh.mesh])

    def saveSceneAsStl(self, stl_name):
        self.__scene.export(self.getPath(stl_name))

    def saveModel(self):
        world_gen = WorldGenerator()
        hg = HeightmapGenerator(map_size=[10, 10, 5])

        # hg.add_layer_from_file(self.getPath('random_map copy1.png'))
        hg.add_custom_layer(self.__image)
        hg.show()
        biome = Biome(n_moisture_zones=1, n_elevation_zones=3)
        biome.add_biome('bottom', color=[203, 65, 107])
        biome.add_biome('middle', color=[6, 154, 243])
        biome.add_biome('top', color=[31, 167, 116])

        biome.add_rule(biome='bottom', moisture_zone=0, elevation_zone=0)
        biome.add_rule(biome='middle', moisture_zone=0, elevation_zone=1)
        biome.add_rule(biome='top', moisture_zone=0, elevation_zone=2)

        # The first blending of textures happends after the first layer ("bottom" biome)
        biome.set_min_height(height=40.0, elevation_zone=1)
        biome.set_fade_dist(fade_dist=1, elevation_zone=1)

        biome.set_min_height(height=80.0, elevation_zone=2)
        biome.set_fade_dist(fade_dist=1, elevation_zone=2)
        hg.biome = biome
        self.__scene = trimesh.Scene([hg.heightmap.mesh.mesh])

        # world_gen.init()
        # world_gen.engines.reset()
        # world_gen.world.add_model(
        #     tag="wall_test",
        #     model=model)
        # fig = plot_occupancy_grid(world_gen.world.models, with_ground_plane=True, static_models_only=False, exclude_contains=['ground_plane'], ground_plane_models=["wall_test"])
       
        # plt.show()


        # world_gen.export_world('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps', 'test')
                


    def showScene(self):
        self.__scene.show()



if __name__ == '__main__':
    MG = MapGenerator()
    MG.getSelfImage()
    MG.generateScene()
    MG.showScene()
    MG.saveSceneAsStl('stuff.stl')
    MG.saveModel()
    MG.showScene()
    MG.saveSceneAsStl('stuff.stl')

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



