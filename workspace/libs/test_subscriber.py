import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
class TestSubscriber:
    def __init__(self):
        rospy.init_node('test', anonymous=False)

        rospy.Subscriber("/target_points", PointCloud, self.receive_id)
    
    def receive_id(self, msg):
        print(msg)

if __name__ == "__main__":
    test_subscriber = TestSubscriber()
    rospy.spin()
