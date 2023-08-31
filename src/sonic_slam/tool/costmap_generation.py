import imp
from PIL import Image
import numpy as np
import sys
import os
import open3d as o3d
ws_path = sys.argv[1]
edgemap = o3d.io.read_point_cloud(os.path.join(ws_path, "data", "map", "edgeMap_pre.pcd"))
surfmap = o3d.io.read_point_cloud(os.path.join(ws_path, "data", "map", "surfaceMap_pre.pcd"))
map = surfmap + edgemap

width = 500
height = 500
# width = 1000
# height = 1000
resolution = 0.1 ####################
map2d_center_x = width / 2
map2d_center_y = height / 2
map3d_center_x = 0
map3d_center_y = 0
occupy_thresh = 7 ###################

costmap = []
for i in range(width):
    costmap.append([])
    for j in range(height):
        costmap[-1].append(0)

count = 0
for point in map.points:
    i = height - int((point[1] - map3d_center_y) / resolution + map2d_center_y)
    j = int((point[0] - map3d_center_x) / resolution + map2d_center_x)
    if(i < 0 or j < 0 or i > width -1  or j > height -1 ):
        continue
    count += 1
    costmap[i][j] += 1
print(count)
for i in range(width):
    for j in range(height):
        if(costmap[i][j] > occupy_thresh):
            costmap[i][j] = 0
        else:
            costmap[i][j] = 255
costmap = np.array(costmap)
img = Image.fromarray(costmap.astype('uint8'))
img.save(os.path.join(ws_path, "data", "map", "costmap.png"))
