import matplotlib.pyplot as plt



class VisualizeMap:
    def __init__(self):
        plt.figure(figsize=(8,8))

    def MAP_show(self, map):
        plt.cla()

        plt.imshow(map)

        plt.pause(0.0001)