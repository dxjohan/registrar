from pyclustering.cluster.optics import optics, ordering_analyser, ordering_visualizer
from pyclustering.utils import read_sample, timedcall
from pyclustering.cluster import cluster_visualizer
import open3d as o3d
import numpy as np
import random
import os

#====== params ======#
MIN_PTS = 100000 # minimum number of points per clusterNode

#====== metadata ======#
dirPath = "C:/Users/Doxel Inc/Documents/registrar/scans/"
numScans = 0
numPts = 0

#====== util functions ======#
def get_float(st):
    if st[0] == '-':
        return -1 * float(st[1:])
    else:
        return float(st)

def reformat(file):
    with open(file) as f:
        content = f.read()
        lines = content.splitlines(True)
        points = []
        for line in lines:
            split_line = line.split(' ')
            points.append([get_float(split_line[0]), get_float(split_line[1]), get_float(split_line[2]), get_float(split_line[3]), get_float(split_line[4]), get_float(split_line[5])])
        return np.array(points)

class ClusterNode:
    def __init__(self, points, depth):
        self.points = points
        self.depth = depth
        self.clusters = []

    def __str__(self):
        return "points: {} depth: {} clusters: {}".format(len(self.points), self.depth, len(self.clusters))

def find_clusters(points):
    n = len(points)
    mid = int(n / 2)
    return [points[:mid], points[mid + 1:]]

def build_tree(points, depth=0):
    if len(points) < MIN_PTS:
        return None
    cluster_node = ClusterNode(points, depth)
    clusters = find_clusters(points)
    for cluster in clusters:
        cluster_node.clusters.append(build_tree(cluster, depth + 1))
    print(cluster_node)
    return cluster_node

#====== build trees ======#
fileInDirList = os.listdir(dirPath)
for file in fileInDirList:
    numScans += 1
    filename = file.split('.')[0]
    outfile = dirPath + file
    points = reformat(outfile)
    print(filename + ':')
    build_tree(points)
    print('')

#====== match clusters ======#

#====== apply transformations ======#

#====== visualize results ======#
