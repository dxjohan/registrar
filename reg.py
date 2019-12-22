import numpy as np
import open3d as o3d
import os

#====== params ======#
MIN_PTS = 100000 # minimum number of points per clusterNode

#====== metadata ======#
numScans = 0
numPts = 0

#====== util functions ======#
def get_float(st):
    if st[0] == '-':
        return -1 * float(st[1:])
    else:
        return float(st)

def find_clusters(points):
    n = len(points)
    mid = int(n / 2)
    return [points[:mid], points[mid + 1:]]

class ClusterNode:
    def __init__(self, points, depth):
        self.points = points
        self.depth = depth
        self.clusters = []

    def __str__(self):
        return "points: {} depth: {} clusters: {}".format(len(self.points), self.depth, len(self.clusters))

def build_cluster(points, depth=0):
    if len(points) < MIN_PTS:
        return None
    clusterNode = ClusterNode(points, depth)
    clusters = find_clusters(points) # list of list
    for cluster in clusters:
        clusterNode.clusters.append(build_cluster(cluster, depth + 1))
    print(clusterNode)
    return clusterNode

#====== load pointclouds & build trees ======#
print('#==== SCANS ====#')
dirPath = "C:/Users/Doxel Inc/Documents/registrar/scans/"
fileInDirList = os.listdir(dirPath)
for file in fileInDirList:
    numScans += 1
    filename = file.split('.')[0]
    outfile = dirPath + file
    with open(outfile) as f:
        content = f.read()
        lines = content.splitlines(True)
        points = []
        for line in lines:
            split_line = line.split(' ')
            points.append([get_float(split_line[0]), get_float(split_line[1]), get_float(split_line[2]), get_float(split_line[3]), get_float(split_line[4]), get_float(split_line[5])])
            numPts += len(points)
        print(filename + ':')
        build_cluster(points)
        print('')

#====== match clusters ======#

#====== apply transformations ======#

#====== visualize results ======#
