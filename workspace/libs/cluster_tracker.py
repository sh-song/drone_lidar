from platform import java_ver
from venv import create
import numpy as np
import pcl
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg    import String
import rospy


class Group:
    id = 0

    ## weight
    w = 1 - np.exp(-1)

    def __init__(self, ref, life):
        Group.id += 1
        self.id = Group.id
        self.life = life

        self.mean_ref = ref
        self.detect_point = ref

        self.mean_tar = ref
        self.target_point = ref

        self.is_spotted = False

        self.highest_point_z = -1

    def update_detect_point(self, new_ref):
        self.detect_point = new_ref

        ## filtering
        self.mean_ref = self.ewma(self.mean_ref, self.detect_point)

    def update_target_point(self, new_target):
        self.target_point = new_target
        
        ## filtering
        self.mean_tar = self.ewma(self.mean_tar, self.target_point)

    ## exponential weighted moving average
    def ewma(self, mean, point):
        mr_x = mean[0]
        mr_y = mean[1]
        mr_z = mean[2]

        rp_x = point[0]
        rp_y = point[1]
        rp_z = point[2]

        mr_x = mr_x * Group.w + rp_x * (1 - Group.w)
        mr_y = mr_y * Group.w + rp_y * (1 - Group.w)
        mr_z = mr_z * Group.w + rp_z * (1 - Group.w)

        return mr_x, mr_y, mr_z

    def get_highest_point(self, cluster):
        for p in cluster:
            pass
class ClusterTracker:
    def __init__(self, params, shared):
        self.params = None
        self.set_params(params)
        self.shared = shared
        self.clusters_queue = [[-2, -2, -2]] 
        self.groups = {}
        self.point_pub = rospy.Publisher("/target_points", PointCloud, queue_size=1)

    def set_params(self, params):
        self.params = params

    def update_data(self):
        current_means_num = len(self.shared.current_means)
        current_queue_num = len(self.clusters_queue)

        if current_means_num > 0:
            if current_queue_num >= self.params['QUEUE_SIZE']:
                for i in range(current_means_num):
                    self.clusters_queue.pop(0) 

            #### z == highest point
            for j, mean in enumerate(self.shared.current_means):
                mean[2] = self.shared.highest_point_list[j]
                self.clusters_queue.append(mean)
            ####
        
        else:
            if current_queue_num > 0:
            #if 0 < current_queue_num <= self.params['QUEUE_SIZE']:
                self.clusters_queue.pop(0) 


    def clustering_in_time(self):
        pcl_data = pcl.PointCloud()
        pcl_data.from_list(self.clusters_queue)
        tree = pcl_data.make_kdtree()
        ec = pcl_data.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.params['CLUSTER_TOLERANCE']) # in meters
        ec.set_MinClusterSize(self.params['CLUSTER_MIN_SIZE']) #min number of points
        ec.set_MaxClusterSize(self.params['CLUSTER_MAX_SIZE']) #max number of points
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()


        for obstacle in cluster_indices:
                
            is_needed_new_group = True
            obs_oldest_point_index = obstacle[0]
            obs_oldest_point = pcl_data[obs_oldest_point_index]

            obs_newest_point_index = obstacle[-1]
            obs_newest_point = pcl_data[obs_newest_point_index]

            #if no group
            if Group.id == 0:
                self.create_new_group(obs_newest_point)

            #check existing groups
            for key, group in self.groups.items():

                ## check distance from reference point
                dist_x = obs_oldest_point[0]-group.detect_point[0]
                dist_y = obs_oldest_point[1]-group.detect_point[1]
                dist = np.hypot(dist_x, dist_y)

                if dist < self.params['MAX_INDIVISUAL_DIST']:
                    group.update_detect_point(obs_oldest_point)
                    group.update_target_point(obs_newest_point)
                    group.is_spotted = True
                    is_needed_new_group = False

                else:
                    pass

            if is_needed_new_group:
                self.create_new_group(obs_newest_point)

        death_note = []

        for id, group in self.groups.items():
            # print(f"id: {id} and group: {group.is_spotted}")
            # print(f"id: {id} and group life: {group.life}")
            # print(f"id: {id} and point: {group.target_point}")

            if group.is_spotted:
                group.life = self.params["MAX_GROUP_LIFE"]

            else:
                group.life -=1
                
            group.is_spotted = False

            if group.life < 0:
                death_note.append(id)

        for id in death_note:
            self.delete_group(id)

    def create_new_group(self, ref):
        new_group = Group(ref, self.params["MAX_GROUP_LIFE"])
        
        self.groups[str(new_group.id)] = new_group


    def delete_group(self, id):
        del(self.groups[id])

    def vis_target_points(self):
        group_id = "0"
        out = PointCloud() #ros msg
        out.header.frame_id = "map"
        channel = ChannelFloat32
        channel.name = "intensity"
        color = []

        for key, group in sorted(self.groups.items()):
            group_id += ", "
            group_id += key        
            point = group.target_point
            # print(group.target_point)
            out.points.append(Point32(point[0], point[1], point[2]))
            color.append(255 * group.id)

        channel.values = color

        out.channels.append(channel)
        self.point_pub.publish(out)          

    def run(self):
        self.update_data()
        self.clustering_in_time()
        self.vis_target_points()
        return self.groups

           
         
