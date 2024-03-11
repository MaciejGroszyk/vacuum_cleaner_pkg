import aspose.words as aw

doc = aw.Document()
builder = aw.DocumentBuilder(doc)

shape = builder.insert_image("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/random_map_rev.png")
shape.get_shape_renderer().save("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/Output.tiff", aw.saving.ImageSaveOptions(aw.SaveFormat.TIFF))
shape.get_shape_renderer().save("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/Output1.tiff", aw.saving.ImageSaveOptions(aw.SaveFormat.TIFF))
import meshlib.mrmeshpy as mr
settings = mr.LoadingTiffSettings()

# load images from specified directory
settings.dir = "/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random"

# specifiy size of 3D image element
settings.voxelSize = mr.Vector3f(1, 1, 5)
#create voxel object from the series of images
volume = mr.loadTiffDir(settings)

#define ISO value to build surface
# iso=127.0
iso=127.0
#convert voxel object to mesh
mesh=mr.gridToMesh(volume, iso)
#save mesh to .stl file
mr.saveMesh(mesh, mr.Path("/home/maciej/vc_ws/src/vacuum_cleaner_pkg/random/stuff.stl"))