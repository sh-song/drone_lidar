import numpy as np



class Map:

    ## YatPal
    def __init__(self):
        self.initialize_map()

    ## for clean memory
    def initialize_map(self):
        previous_map = np.zeros((9,9))

        try:
            previous_map = self.LocalMap
            del(self.LocalMap)

        except:
            pass

        self.LocalMap = np.zeros((9,9))
        self.LocalMap[4,4] = -1

        return previous_map

    ## get group dictionary which has group instances
    def represent_target_points(self, groups):
        for key, point in groups.items():
            x, y = self.relative_target_position(point)
            if (x == 4) and (y == 4):
                pass
            else:
                self.LocalMap[x, y] = int(key)
            
    ## target point's position relative to drone
    def relative_target_position(self, point):
        x = point.target_point[0]
        y = point.target_point[1]

        ## shift x, y: -18 ~ 18 -> 0 ~ 36
        x -= 18
        y -= 18

        ## LocalMap is (9,9) matrix so chop point (36m, 36m)
        ## by size (4m, 4m)
        x_ = int(abs(x) / 4)
        y_ = int(abs(y) / 4)

        return x_, y_

    def run(self, groups):

        previous_map = self.initialize_map()
        self.represent_target_points(groups)

        if not (previous_map == self.LocalMap).all():
            print(self.LocalMap)

            for key, group in groups.items():
                print(group.target_point)
        
