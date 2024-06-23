
import aspose.words as aw
import meshlib.mrmeshpy as mr

import cv2
import json
import os
import random
import ctypes 
from distutils.dir_util import copy_tree

import json

class WorldMapModelGenerator():

    def __init__(self):

        self.__distanceMap = None
        self.__current_file_path   = os.path.abspath(os.path.dirname(__file__))
        self.__package_path        = os.path.dirname(self.__current_file_path)

        self.MIN = 1
        self.MAX = 1968

        self.PSEUDO_MAP_FOLDER  = "/data/pseudo_random_maps"
        self.LOCAL_MODEL_PATH   = "/data/map_model"
        self.CURRENT_MODEL_PNG  = "/data/map_model/current_map_img.png"
        self.MODEL_TIFF_FOLDER  = "/data/map_model/tiff_files"


    def __readFileImage(self, imageFilePath) -> mr.DistanceMap:
        return mr.loadDistanceMapFromImage(mr.Path(imageFilePath), 0)

    def __generateMapModelStl(self, image_path):
        doc = aw.Document()
        builder = aw.DocumentBuilder(doc)
        shape = builder.insert_image(image_path)

        tiff_path = self.__package_path + self.MODEL_TIFF_FOLDER 
        shape.get_shape_renderer().save(tiff_path + "/tiff1.tiff", aw.saving.ImageSaveOptions(aw.SaveFormat.TIFF))
        shape.get_shape_renderer().save(tiff_path + "/tiff2.tiff", aw.saving.ImageSaveOptions(aw.SaveFormat.TIFF))

        settings = mr.LoadingTiffSettings()
        settings.dir = tiff_path

        settings.voxelSize = mr.Vector3f(1, 1, 5)
        volume = mr.loadTiffDir(settings)

        iso=127.0
        mesh=mr.gridToMesh(volume, iso)
        stl_path = self.__package_path + self.LOCAL_MODEL_PATH
        mr.saveMesh(mesh, mr.Path(stl_path  + "/my_model.stl"))
    
        box3f = mesh.getBoundingBox()
        self.saveStlParam(box3f.center())
        return box3f.center().x*2, box3f.center().y*2


    def saveStlParam(self, center):
        dict = {
            "width": center.y * 2,
            "height": center.x * 2
        }
        json_object = json.dumps(dict, indent=2)
        
        with open(self.__package_path+self.LOCAL_MODEL_PATH+"/stl_size.json", "w") as outfile:
            outfile.write(json_object)

    def getPseudoRandomMapFilePath(self) -> os.path:
        random_id = self.__getRandomMapFileId()
        formatted_id = "{:04d}".format(random_id)
        return self.__package_path + self.PSEUDO_MAP_FOLDER + "/" + str(formatted_id) + ".png"

    def getMapFilePath(self, id : int) -> os.path:
        formatted_id = "{:04d}".format(id)
        return self.__package_path + self.PSEUDO_MAP_FOLDER + "/" + str(formatted_id) + ".png"

    def __getRandomIntFromRange(self, min: int, max : int) -> int:
        return random.randint(min, max)
    
    def __getRandomMapFileId(self):
        return self.__getRandomIntFromRange(self.MIN, self.MAX)

    def getCurrentFilePath(self) -> os.path:
        return self.__current_file_path

    def getPackagePath(self) -> os.path:
        return self.__package_path      

    def readRandomImage(self):
        return cv2.imread(self.getPseudoRandomMapFilePath())

    def readImageId(self, id:int):
        return cv2.imread(self.getMapFilePath(id))

    def saveImage(self, image):
        path = self.getPackagePath() + self.CURRENT_MODEL_PNG
        cv2.imwrite(path, image)
        return path

    def getNegatedImage(self, img):
        return cv2.bitwise_not(img)

    def worldMapModelGenerator(self):
        random_img = self.readRandomImage()
        neg_random_img = self.getNegatedImage(random_img)
        path_neg_random_img = self.saveImage(neg_random_img)

        w, h = self.__generateMapModelStl(path_neg_random_img)
        
        resized = cv2.resize(neg_random_img, (int(w), int(h)))
        self.saveImage(resized)

    
    def worldMapModelGeneratorId(self, id :int):
        img = self.readImageId(id)
        neg_img = self.getNegatedImage(img)
        path_img = self.saveImage(neg_img)

        w, h = self.__generateMapModelStl(path_img)

        resized = cv2.resize(neg_img, (int(w), int(h)))
        self.saveImage(resized)

    def worldFromPngModelGenerator(self, img):
        neg_random_img = self.getNegatedImage(img)
        path_neg_random_img = self.saveImage(neg_random_img)

        self.__generateMapModelStl(path_neg_random_img)

    def sendGeneratedMapModelToGazeboModelPath(self):
        copy_folder_path = self.__package_path + self.LOCAL_MODEL_PATH
        folder_out_path = "~/.gazebo/models"
        print(folder_out_path)
        os.system('cp -r ' + copy_folder_path + ' ' + folder_out_path)
        # copy_tree(copy_folder_path, folder_out_path)


if __name__ == "__main__":
    wmmg = WorldMapModelGenerator()
    print(wmmg.getCurrentFilePath())
    print(wmmg.getPackagePath())
    print(wmmg.getPseudoRandomMapFilePath())

    # wmmg.worldMapModelGenerator()
    wmmg.worldMapModelGeneratorId(5)
    wmmg.sendGeneratedMapModelToGazeboModelPath()
