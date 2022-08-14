import numpy as np
import pcl

class ClusterTracker:
    def __init__(self, params, shared):
        self.params = params
        self.shared = shared
        self.clusters_queue = [[-2, -2, -2]] 
        self.id = {}
        self.target_positions = {'0':[], '1':[], '2':[]}
        self.reference_points = {'0':[], '1':[], '2':[]}
    def update_data(self):
        current_means_num = len(self.shared.current_means)
        if len(self.shared.current_means) > 0:
            if len(self.clusters_queue) >= self.params['QUEUE_SIZE']:
                for i in range(current_means_num):
                    self.clusters_queue.pop(0) 
            for mean in self.shared.current_means:
                self.clusters_queue.append(mean)

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
            print('------------')
            for obstacle in cluster_indices:
                obs_oldest_point_index = obstacle[0]
                obs_oldest_point = pcl_data[obs_oldest_point_index]

                obs_newest_point_index = obstacle[-1]
                obs_newest_point = pcl_data[obs_newest_point_index]

                for key, ref_point in self.reference_points.items():
                    if np.isclose(obs_oldest_point, ref_point):
                        self.target_positions[key] = obs_newest_point

    def set_reference_point(self):
        #TODO
        pass
    def run(self):
            self.update_data()
            self.clustering_in_time()

            # print(len(self.clusters_queue), 'len')

           
         