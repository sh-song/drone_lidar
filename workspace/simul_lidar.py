import socket
import numpy as np
import time
import math
import os,json
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import socket
import threading
from sig_int_handler import Activate_Signal_Interrupt_Handler

user_ip = "127.0.0.1"
host_ip = "127.0.0.1"
lidar_port = 2368
#"lidar_host_port" : 2369,
# "lidar_dst_port" : 2368,

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 32, #verticla channel of a lidar
    "localIP": user_ip,
    "hostIP" : host_ip,
    "localPort": lidar_port,
    "Block_SIZE": int(1206)
}

def RotationMatrix(yaw, pitch, roll):
     
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
                 
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R


def TranslationMatrix(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M


class UDP_LIDAR_Parser :
    
    def __init__(self, ip, port, params_lidar=None):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)

        self.data_size=params_lidar["Block_SIZE"]
        
        self.x, self.y, self.z = [], [], []
        if params_lidar["CHANNEL"]==int(16):
            self.channel = int(16)
            self.max_len = 150
            self.VerticalAngleDeg = np.array([[-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]])

        else:
            self.channel = int(32)
            self.max_len = 300
            self.VerticalAngleDeg = np.array([[-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
                                    0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]])


        self.is_lidar=False
        thread = threading.Thread(target=self.loop)
        thread.daemon = True 
        thread.start() 
    
    def loop(self):
        while True:
            print('----------')
            x, y, z, intensity = self.recv_udp_data()
            self.x,self.y,self.z,self.Intensity= x, y, z, intensity 
            self.is_lidar=True

    # def lidar_result(self,x,y,z,Intensity):
    #     return x,y,z,Intensity


    def recv_udp_data(self):

        Buffer = b''

        for _ in range(self.max_len):

            UnitBlock, sender = self.sock.recvfrom(self.data_size)
            
            Buffer+=UnitBlock[:1200]

        Buffer_np=np.frombuffer(Buffer, dtype=np.uint8).reshape([-1, 100])

        if self.channel==16:
            Azimuth = np.zeros((24*self.max_len,))
            Azimuth[0::2] = Buffer_np[:,2].astype(np.float32) + 256*Buffer_np[:,3].astype(np.float32)
            Azimuth[1::2] = Buffer_np[:,2].astype(np.float32) + 256*Buffer_np[:,3].astype(np.float32) + 20
        else:
            Azimuth = Buffer_np[:,2] + 256*Buffer_np[:,3]
        
        Distance = (Buffer_np[:,4::3].astype(np.float32) + 256*Buffer_np[:,5::3].astype(np.float32))*2
        Intensity = Buffer_np[:,6::3].astype(np.float32)

        # reshape outputs based on 16 channels
        Azimuth = Azimuth.reshape([-1, 1])/100
        Distance = Distance.reshape([-1, self.channel])/1000
        Intensity = Intensity.reshape([-1])

        x, y, z = self.sph2cart(Distance, Azimuth)

        return x, y, z, Intensity

    def sph2cart(self, R, a):

        x = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.sin(np.deg2rad(a))
        y = R * np.cos(np.deg2rad(self.VerticalAngleDeg)) * np.cos(np.deg2rad(a))
        z = R * np.sin(np.deg2rad(self.VerticalAngleDeg))
        
        return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])

    def __del__(self):
        self.sock.close()
        print('del')


def main():

    udp_lidar = UDP_LIDAR_Parser(ip=params_lidar["hostIP"], port=params_lidar["localPort"], params_lidar=params_lidar)

    rospy.init_node('simul_lidar_parser', anonymous=False)

    pc_pub = rospy.Publisher("/simul_velodyne_points", PointCloud, queue_size=1)

    while True :

        if udp_lidar.is_lidar ==True:            
            x=udp_lidar.x
            y=udp_lidar.y
            z=udp_lidar.z
            intensity=udp_lidar.Intensity 

            pc_msg = PointCloud()
            pc_msg.header.frame_id = "map"

            step = 4
            for n in range(step):

                pc_msg = PointCloud()
                for i in range(n, (n+1)*(115200//step)):
                    pc_msg.points.append(Point32(x[i], y[i], z[i]))
                pc_pub.publish(pc_msg)
                
            # for i in range(57600):
            #     pc_msg.points.append(Point32(x[i], y[i], z[i]))

            # pc_pub.publish(pc_msg)

            # pc_msg = PointCloud()
            # for i in range(57600, 115200):
            #     pc_msg.points.append(Point32(x[i], y[i], z[i]))

            # pc_pub.publish(pc_msg)



            print('-------------------------')
                # xyz1 = np.concatenate([
                #     x.reshape([-1, 1]),
                #     y.reshape([-1, 1]),
                #     z.reshape([-1, 1])
                # ], axis=1).T.astype(np.float32)

                # print(xyz1.T)







    
if __name__ == '__main__':
    Activate_Signal_Interrupt_Handler()

    main()

