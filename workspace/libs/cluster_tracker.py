from platform import java_ver
from venv import create
import numpy as np
import pcl
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import rospy

class Group:
    id = -1
    def __init__(self, ref, life):
        Group.id += 1
        self.id = Group.id
        self.life = life

        self.reference_point = ref
        self.target_point = ref
        self.is_spotted = False

    def update_reference_point(self, new_ref):
        self.reference_point = new_ref
    def update_target_point(self, new_target):
        self.target_point = new_target


class ClusterTracker:
    def __init__(self, params, shared):
        self.params = params
        self.shared = shared
        self.clusters_queue = [[-2, -2, -2]] 
        self.groups = {}
    
        self.max_group_life = self.params['MAX_GROUP_LIFE'] 
        self.point_pub = rospy.Publisher("/target_points", PointCloud, queue_size=1)

    def update_data(self):
        current_means_num = len(self.shared.current_means)
        if len(self.shared.current_means) > 0:
            if len(self.clusters_queue) >= self.params['QUEUE_SIZE']:
                for i in range(current_means_num):
                    self.clusters_queue.pop(0) 
            for mean in self.shared.current_means:
                self.clusters_queue.append(mean)
        else:
            if 0 < len(self.clusters_queue) <= self.params['QUEUE_SIZE']:
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
            # print(cluster_indices)


            for obstacle in cluster_indices:
                
                is_needed_new_group = False
                obs_oldest_point_index = obstacle[0]
                obs_oldest_point = pcl_data[obs_oldest_point_index]

                obs_newest_point_index = obstacle[-1]
                obs_newest_point = pcl_data[obs_newest_point_index]

                #if no group
                if Group.id == -1:
                    self.create_new_group(obs_newest_point)
                else:
                    print('group size',len(self.groups.keys()))
                    if len(self.groups.keys()) == 0:
                        is_needed_new_group = True
                    else:
                        is_needed_new_group = False
                #check existing groups
                for key, group in self.groups.items():
                    dist = np.hypot((obs_oldest_point[0] - group.reference_point[0]),(obs_oldest_point[1] - group.reference_point[1]))
                    if dist < 1:
                    #if np.isclose(obs_oldest_point, group.reference_point).all():
                        group.update_reference_point(obs_oldest_point)
                        group.update_target_point(obs_newest_point)
                        group.is_spotted = True
                    else:
                        is_needed_new_group = True
                        group.is_spotted = False

                if is_needed_new_group:
                    self.create_new_group(obs_newest_point)
                
            #check is dead 
            death_note = []
            for id, group in self.groups.items():

                print(f"id: {id} and group: {group.life}")
                if group.is_spotted:
                    group.life = self.max_group_life
                else:

                    group.life -=1
                    
                group.is_spotted = False
                if group.life < 0:
                    death_note.append(id)
            for id in death_note:
                self.delete_group(id)


    def create_new_group(self, ref):
        new_group = Group(ref, self.max_group_life)
        
        self.groups[str(new_group.id)] = new_group

    def delete_group(self, id):
        del(self.groups[id])

    def vis_target_points(self):
        out = PointCloud() #ros msg
        out.header.frame_id = "map"
        cnt = 0
        for group in self.groups.values():
            point = group.target_point
            out.points.append(Point32(point[0], point[1], point[2]))
            cnt +=1
        self.point_pub.publish(out)              


    def run(self):
            self.update_data()
            self.clustering_in_time()
            self.vis_target_points()
            # print(len(self.clusters_queue), 'len')

           
         