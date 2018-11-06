"""
This file is given to the students for the SLAM project. It reads map data from map_data.txt and plays it through
slam_helper.py
"""

# most amazing and greatest python library
import ast
from student_slam_helper import FastSLAM
import numpy as np

map_path = 'map_data.txt'

file = open(map_path, 'r')

map_data = file.readline()
z_data = file.readline()

unprocessed_map_data = ast.literal_eval(map_data)
z_data = ast.literal_eval(z_data)

# chop map_data so same length as z_data
unprocessed_map_data = unprocessed_map_data[0:694]

map_data = []
for data in unprocessed_map_data:
    des_list = np.asarray(data[1], dtype=np.uint8)
    map_data.append((data[0], des_list))

print map_data[0][1]

slam_estimator = FastSLAM()
slam_estimator.generate_particles(15)

for i in range(1, len(map_data)):
    print slam_estimator.run(z_data[i-1], map_data[i-1][0], map_data[i-1][1], map_data[i][0], map_data[i][1])








