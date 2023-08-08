import pcl
import numpy as np

cloud = pcl.load('/root/catkin_ws/src/sonic_localization/data/map/dating.pcd')
cloud = np.asarray(cloud)

print(cloud.shape)
