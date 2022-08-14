self.MAP_SIZE = params['MAP_SIZE'] #m, square
self.VOXEL_SIZE = params['VOXEL_SIZE'] #m, square
self.EGO_SIZE = params['EGO_SIZE'] #m, square
self.CLUSTER_TOLERANCE = params['CLUSTER_TOLERANCE'] # in meters
        #cluster_extraction is a greedy region growing algorithm 
        #based on nearest neighbors.
        #Cluster affinity is based on a distance to 
        #any point of a cluster (cluster tolerance parameter).
self.CLUSTER_MIN_SIZE = params['CLUSTER_MIN_SIZE'] #min number of points
self.CLUSTER_MAX_SIZE = params['CLUSTER_MAX_SIZE'] #max number of points      
self.VERTICAL_UPPER_ROI = params['VERTICAL_UPPER_ROI'] #m
self.VERTICAL_LOWER_ROI = params['VERTICAL_LOWER_ROI'] #m

