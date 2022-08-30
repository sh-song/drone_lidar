from ast import AsyncFunctionDef
import os,json
from csv import reader
from pydoc import cli
from types import DynamicClassAttribute
import rospy
import numpy as np
import sys
from sensor_msgs.msg import PointCloud2
import time
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import String
import struct
import argparse
import pcl
import threading
from scipy.spatial import ConvexHull
from libs.cluster_tracker import ClusterTracker
from libs.i_am_map        import Map
# from libs.visualize_map   import VisualizeMap
from libs.sig_int_handler import Activate_Signal_Interrupt_Handler
np.set_printoptions(threshold=sys.maxsize)

class Shared:
    def __init__(self):
        self.current_means = None
        self.cluster_cloud_list = None
        self.highest_point_list = None

class PCParser:
    def __init__(self, args, params, hz):
        print('init..... if using simulator, add "--lidar simul" argument')
        rospy.init_node('parser', anonymous=False)
        self.platform_params = params
        self.params = self.platform_params['building']
        self.rate = rospy.Rate(hz)
        if args.lidar == 'simul':
            rospy.Subscriber("/lidar3D", PointCloud2, self.ros_to_pcl)
        else:
            rospy.Subscriber("/velodyne_points", PointCloud2, self.ros_to_pcl)
        
        self.point_pub = rospy.Publisher("/processed_cloud", PointCloud, queue_size=1)
        self.cluster_pub = rospy.Publisher("/cluster", PointCloud, queue_size=1)
       
        self.shared = Shared()
        self.pcl_data = pcl.PointCloud()
        self.new_pcl_data = pcl.PointCloud()
        self.voxelized_data = None
        self.roi_cropped_data = pcl.PointCloud()
        
        self.tracker = ClusterTracker(self.params['tracker'], self.shared)

			
        self.target_waypoint = "-1"
        rospy.Subscriber("/target_waypoint", String, self.waypoint_cb)

 
    def ros_to_pcl(self, msg): #in sub thread
        points_list = []
        for data in pc2.read_points(msg, skip_nans=True):
            points_list.append([data[0], data[1], data[2]])

        self.new_pcl_data = pcl.PointCloud()
        self.new_pcl_data.from_list(points_list)
        #print('callback', time.time())
    def waypoint_cb(self, msg):
        self.target_waypoint = msg.data
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
        vertical_roi = self.do_passthrough(passthrough_filter, "z", self.params['VERTICAL_LOWER_ROI'], self.params['VERTICAL_UPPER_ROI'], is_negative=False)

        passthrough_filter = vertical_roi.make_passthrough_filter()
        front_roi = self.do_passthrough(passthrough_filter, "y", roi_min, roi_max, is_negative=False)
        rear_roi = self.do_passthrough(passthrough_filter, "y", roi_min, roi_max, is_negative=True)

        left_roi = self.do_passthrough(passthrough_filter, "x", 0, 0, is_negative=True)
        right_roi = self.do_passthrough(passthrough_filter, "x", roi_min, roi_max, is_negative=False)

        passthrough_filter = right_roi.make_passthrough_filter()
        right_roi = self.do_passthrough(passthrough_filter, "y", -roi_max, roi_max, is_negative=False)

        passthrough_filter = front_roi.make_passthrough_filter()
        front_roi = self.do_passthrough(passthrough_filter, "x", 0, roi_min, is_negative=False)

        passthrough_filter = rear_roi.make_passthrough_filter()
        rear_roi = self.do_passthrough(passthrough_filter, "x", 0, roi_min, is_negative=False)


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
        # print(self.roi_cropped_data.size)
        # print(self.voxelized_data.size)

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

        ec.set_ClusterTolerance(self.params['CLUSTER_TOLERANCE'])
        ec.set_MinClusterSize(self.params['CLUSTER_MIN_SIZE']) #min number of points
        ec.set_MaxClusterSize(self.params['CLUSTER_MAX_SIZE']) #max number of points
        ec.set_SearchMethod(tree)

        cluster_indices = ec.Extract()
        # print(cluster_indices)
        #cluster_color = self.get_color_list(len(cluster_indices))
        cluster_cloud_list = []
        current_means = []
        highest_point_list = []
        for j, indices in enumerate(cluster_indices): #jth cluster
            cluster_with_color = []
            for i, indice in enumerate(indices):
                cluster_with_color.append([self.voxelized_data[indice][0],
                                                self.voxelized_data[indice][1],
                                                self.voxelized_data[indice][2],
                                                self.rgb_to_float([255,0,0])])

            cluster_cloud = np.array(cluster_with_color)
            cluster_cloud_list.append(cluster_cloud)

            mean = np.mean(cluster_cloud, axis=0)
            current_means.append(mean[0:3])

            highest_point = np.max(cluster_cloud[:,2])
            highest_point_list.append(highest_point)

        self.shared.cluster_cloud_list = cluster_cloud_list
        self.shared.current_means = current_means
        self.shared.highest_point_list = highest_point_list


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
        self.cluster_number = len(self.shared.cluster_cloud_list)
        for i, cluster in enumerate(self.shared.cluster_cloud_list):
            color_constant = 1/self.cluster_number 
            for p in cluster:
                out.points.append(Point32(p[0], p[1], p[2]))
        self.cluster_pub.publish(out)        

    def target_publish(self,groups):
        print(groups)

        
    def run(self):
        while not rospy.is_shutdown():
            #Update Data
            self.pcl_data = self.new_pcl_data

            if self.target_waypoint == "2":
                self.params = self.platform_params['pillars']
            else:
                self.params = self.platform_params['building']


            #Preprocessing
            self.roi_cropping(roi_min = self.params['EGO_SIZE'], roi_max = self.params['MAP_SIZE'])
            self.voxelize(self.params['VOXEL_SIZE'])

            #Clustering
            self.euclidean_clustering()
            #Tracking
            self.tracker.set_params(self.params['tracker'])
            tracked_points = self.tracker.run()
            self.target_publish(tracked_points)
            self.visualize_cluster()
            self.visualize("voxel")

            self.rate.sleep()
if __name__ == "__main__":
    Activate_Signal_Interrupt_Handler()

    argparser = argparse.ArgumentParser(
        description="ask shs"
    )

    argparser.add_argument(
        '--lidar',
        default='real',
        help='simul or real'
    )

    argparser.add_argument(
        '--platform',
        default='gigacha',
        help='gigacha or dok3'
    )

    path = os.path.dirname( os.path.abspath( __file__ ) )

    with open(os.path.join(path,("params.json")),'r') as fp :
        params = json.load(fp)
    
    args = argparser.parse_args()
    if args.platform == "gigacha":
        params = params['gigacha']
        print('platform: gigacha')

    elif args.platform == "dok3":
        params = params['dok3']
        print('platform: dok3')

    pp = PCParser(args, params, hz=20)
    pp.run()
