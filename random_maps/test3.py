import  jpype     
import  asposecells     
jpype.startJVM() 
from asposecells.api import Workbook
from PIL import Image, ImageOps

image = Image.open('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map.png')

inverted_image = ImageOps.invert(image)

inverted_image.save('/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map_rev.png')
workbook = Workbook("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map.png")
workbook.save("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/Output.tiff")
jpype.shutdownJVM()

import meshlib.mrmeshpy as mr
settings = mr.LoadingTiffSettings()

# load images from specified directory
settings.dir = "/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random"

# specifiy size of 3D image element
settings.voxelSize = mr.Vector3f(1, 1, 1)

#create voxel object from the series of images
volume = mr.loadTiffDir(settings)

#define ISO value to build surface
iso=127.0

#convert voxel object to mesh
mesh=mr.gridToMesh(volume, iso)

#save mesh to .stl file
mr.saveMesh(mesh, mr.Path("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/stuff.stl"))