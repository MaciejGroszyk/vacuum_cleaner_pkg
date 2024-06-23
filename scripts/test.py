import cv2
import numpy as np

img= cv2.imread("/home/maciej/vc_ws/src/random_world_generator_pkg/data/map_model/current_map_img_with_path.png")

rows,cols,_ = img.shape

count_white = 0
count_blue = 0
count_black = 0
for i in range(rows):
    for j in range(cols):
        k = img[i,j]
        if  k[0] == 255 and k[1] == 255 and k[2] == 255:
            count_black += 1
        elif k[0] == 255 and k[1] == 0 and k[2] == 0:
            count_blue += 1
        elif k[0] == 0 and k[0] == 0 and k[2] == 0:
            count_white += 1
r, c, _ = img.shape
print("ALL:" + str(r * c))
print("Count black:" + str(count_black))
print("Count blue :" + str(count_blue))
print("Count white:" + str(count_white))
print("Count black:" + str((r * c)-count_blue -cv2.countNonZero(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))))
print("Count blue %:"+ str(100*count_blue/(count_blue+count_black)))     

binary_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
print(cv2.countNonZero(binary_img))