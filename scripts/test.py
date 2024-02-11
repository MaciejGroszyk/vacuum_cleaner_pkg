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

final_image = Image.open('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map.png')
final_image = final_image.resize((65, 65))
# Normalize the image with values between 0 and 255
normalized_image = (final_image - np.min(final_image)) * (255.0 / (np.max(final_image) - np.min(final_image)))
normalized_image = normalized_image.astype('uint8')

# Convert the normalized numpy array to an image using Pillow
image = pillow.Image.fromarray(normalized_image)

image.save('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map1.png')
image = Image.open('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map.png')
# Create a heightmap generator
hg = HeightmapGenerator(
    map_size=[10, 10, 2])
# hg.add_perlin_noise_layer(freq=110.0, octaves=10)
hg.add_layer_from_file("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/random_map copy.png")

scene = trimesh.Scene([hg.heightmap.mesh.mesh])
# scene.show()

scene.export('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random_maps/stuff.stl')

