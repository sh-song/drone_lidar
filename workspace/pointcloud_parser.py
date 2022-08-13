from ast import AsyncFunctionDef
from csv import reader
from pydoc import cli
import rospy
import numpy as np
import sys
from sensor_msgs.msg import PointCloud2
import time
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from sig_int_handler import Activate_Signal_Interrupt_Handler
import struct
import argparse
from random import randint
import pcl
import threading
from scipy.spatial import ConvexHull
from simul_lidar import UDP_LIDAR_Parser
np.set_printoptions(threshold=sys.maxsize)

class PCParser:
    def __init__(self, args):
        print('init.....')
        rospy.init_node('parser', anonymous=False)
        if args.lidar == 'simul':
            rospy.Subscriber("/lidar3D", PointCloud2, self.ros_to_pcl)
        else:
            rospy.Subscriber("/velodyne_points", PointCloud2, self.ros_to_pcl)
        
        self.point_pub = rospy.Publisher("/processed_cloud", PointCloud, queue_size=1)
        self.cluster_pub = rospy.Publisher("/cluster", PointCloud, queue_size=1)

        self.pcl_data = pcl.PointCloud()
        self.new_pcl_data = pcl.PointCloud()
        self.voxelized_data = None
        self.roi_cropped_data = pcl.PointCloud()
        
        self.cluster_number = 0
        self.cluster_cloud_list = None
        self.MAP_SIZE = 50.0 #m, square
        self.VOXEL_SIZE = 0.2 #m, square
        self.EGO_SIZE = 0.5 #m, square
        self.GRIDS_PER_EDGE = int(self.MAP_SIZE // self.VOXEL_SIZE)
        self.grid_map = np.zeros([self.GRIDS_PER_EDGE, self.GRIDS_PER_EDGE])

 
    def ros_to_pcl(self, msg): #in sub thread
        points_list = []
        for data in pc2.read_points(msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        self.new_pcl_data = pcl.PointCloud()
        self.new_pcl_data.from_list(points_list)
        #print('callback', time.time())

    # ROI
    def do_passthrough(self, passthrough_filter, filter_axis="x", roi_min=0.5, roi_max=15.0, is_negative=False):
        
        if is_negative:
            temp = roi_min
            roi_min = -roi_max
            roi_max = -temp
            
        passthrough_filter.set_filter_field_name(filter_axis)
        passthrough_filter.set_filter_limits(roi_min, roi_max)
        return passthrough_filter.filter()

    def roi_cropping(self,roi_min=0.5, roi_max=15):
    
        passthrough_filter = self.pcl_data.make_passthrough_filter()
        
        front_roi = self.do_passthrough(passthrough_filter, "y", roi_min, roi_max, is_negative=False)
        rear_roi = self.do_passthrough(passthrough_filter, "y", roi_min, roi_max, is_negative=True)
        
        left_roi = self.do_passthrough(passthrough_filter, "x", roi_min, roi_max, is_negative=True)
        right_roi = self.do_passthrough(passthrough_filter, "x", roi_min, roi_max, is_negative=False)

        passthrough_filter = left_roi.make_passthrough_filter()
        left_roi = self.do_passthrough(passthrough_filter, "y", -roi_min, roi_min, is_negative=False)

        passthrough_filter = right_roi.make_passthrough_filter()
        right_roi = self.do_passthrough(passthrough_filter, "y", -roi_min, roi_min, is_negative=False)

        passthrough_filter = front_roi.make_passthrough_filter()
        front_roi = self.do_passthrough(passthrough_filter, "x", -roi_max, roi_max, is_negative=False)

        passthrough_filter = rear_roi.make_passthrough_filter()
        rear_roi = self.do_passthrough(passthrough_filter, "x", -roi_max, roi_max, is_negative=False)
        


        if front_roi.to_array().size == 0:
           front_roi = np.zeros((1,3), dtype=np.float32)
        if rear_roi.to_array().size == 0:
           rear_roi = np.zeros((1,3), dtype=np.float32)
        if left_roi.to_array().size == 0:
           left_roi = np.zeros((1,3), dtype=np.float32)
        if right_roi.to_array().size == 0:
           right_roi = np.zeros((1,3), dtype=np.float32)

        try:
            longitudinal_roi = np.concatenate([front_roi, rear_roi], axis=0)
            lateral_roi = np.concatenate([right_roi, left_roi], axis=0)
            total_roi = np.concatenate([longitudinal_roi, lateral_roi], axis=0)

            new_pcl_data = pcl.PointCloud()
            new_pcl_data.from_array(total_roi)
            self.roi_cropped_data = new_pcl_data

        except ValueError as e:
            print(e)
            self.roi_cropped_data = pcl.PointCloud()

    def voxelize(self, leaf_size=1): #m

        vox = self.roi_cropped_data.make_voxel_grid_filter()
        vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
        self.voxelized_data = vox.filter()

    def rgb_to_float(self, color):
        hex_r = (0xff & color[0]) << 16
        hex_g = (0xff & color[1]) << 8
        hex_b = (0xff & color[2])

        hex_rgb = hex_r | hex_g | hex_b

        float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

        return float_rgb

    def euclidean_clustering(self):

        tree = self.voxelized_data.make_kdtree()
        # Create Cluster-Mask Point Cloud to visualize each cluster separately
        ec = self.voxelized_data.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.5) # in meters
        ec.set_MinClusterSize(10) #min number of points
        ec.set_MaxClusterSize(200) #max number of points
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        #cluster_color = self.get_color_list(len(cluster_indices))
        cluster_cloud_list = []
        self.cluster_number = len(cluster_indices)
        # print('#ofclusters', len(cluster_indices))
        for j, indices in enumerate(cluster_indices): #jth cluster
            cluster_with_color = []
            for i, indice in enumerate(indices):
                cluster_with_color.append([self.voxelized_data[indice][0],
                                                self.voxelized_data[indice][1],
                                                self.voxelized_data[indice][2],
                                                self.rgb_to_float([255,0,0])])

            # cluster_cloud = pcl.PointCloud_PointXYZRGB()
            # cluster_cloud.from_list(cluster_with_color)
            cluster_cloud = np.array(cluster_with_color)
            cluster_cloud_list.append(cluster_cloud)

        print('eee', self.cluster_number)
        self.cluster_cloud_list = cluster_cloud_list

    def visualize(self, target, data=None):
        if target=="raw":
            points = self.pcl_data
        elif target=="roi":
            points = self.roi_cropped_data
        elif target=="voxel":
            points = self.voxelized_data

        out = PointCloud() #ros msg
        out.header.frame_id = "map"
        for p in points:
            out.points.append(Point32(p[0], p[1], p[2]))
        self.point_pub.publish(out)        

    def visualize_cluster(self):
        out = PointCloud()
        out.header.frame_id = "map"
        channel = ChannelFloat32
        channel.name = "intensity"
        color = []
        for i, cluster in enumerate(self.cluster_cloud_list):
            color_constant = 1/self.cluster_number 
            for p in cluster:
                out.points.append(Point32(p[0], p[1], p[2]))

                # out.points.append(Point32(p[0], p[1], p[2]))
                color.append(i*color_constant)
            print('----------', cluster)
        channel.values = color
        out.channels.append(channel)
        self.cluster_pub.publish(out)        

    def cluster_filling(self):
        new_grid_map = np.zeros([self.GRIDS_PER_EDGE, self.GRIDS_PER_EDGE])
        for i, cluster in enumerate(self.cluster_cloud_list):
            hull_cluster = cluster[:, 0:2]
            print('cluster', cluster[:, 0:2].shape)
            print('hull', hull_cluster.shape)
            print('--------------')
            for p in hull_cluster:
                # print(int(p[0]/self.VOXEL_SIZE), int(p[1]/self.VOXEL_SIZE))
                new_grid_map[int(p[0]/self.VOXEL_SIZE), int(p[1]/self.VOXEL_SIZE)] = 1

        print(new_grid_map)

        # self.filled_cluster_cloud_list = filled_cluster_cloud_list
    def run(self):
        while True:
            self.pcl_data = self.new_pcl_data
            self.roi_cropping(roi_min = self.EGO_SIZE, roi_max = self.MAP_SIZE)
            self.voxelize(self.VOXEL_SIZE)
            self.euclidean_clustering()
            # self.cluster_filling()
            self.visualize_cluster()
            self.visualize("voxel")
                


if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()

    argparser = argparse.ArgumentParser(
        description="DOK3"
    )
    argparser.add_argument(
        '--lidar',
        default='real',
        help='simul or real'
    )

    args = argparser.parse_args()
    pp = PCParser(args)
    
    pp.run()
