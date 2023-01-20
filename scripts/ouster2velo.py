#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import numpy as np
import argparse
import time

class Ouster2Velo:
    def __init__(self, args, opts):
        
        # read input and output topics
        self.in_topic = args.in_topic
        self.out_topic = args.out_topic
        
        self.sub = rospy.Subscriber(self.in_topic, PointCloud2, self.callback)
        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1000)

    def callback(self, msg):
        start_time = time.time()
        points_list = []
        # ouster cloud format
        #  0 1 2    3      4      5         6       7       8
        # {x,y,z,intensity,t,reflectivity,ring, ambient, range}
        for data in pcl2.read_points(msg, skip_nans=True):
            # if data[4] > 0:
            #     print('')
            points_list.append(data)

        pc = np.array(points_list).astype(np.float32)

        # velo cloud format
        #  0 1 2    3       4      5 
        # {x,y,z,intensity,time,ring}
        pc_velo = pc[:, [0, 1, 2, 3, 4, 6]]
        pc_velo[:, 4] *= 1e-3   # velo expects time in microseconds

        # create header
        header = Header()
        header.frame_id = 'velodyne' #msg.header.frame_id
        header.stamp = msg.header.stamp #TODO nano seconds to microseconds?
        # fill pcl msg
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('intensity', 12, PointField.FLOAT32, 1),
                    PointField('time', 16, PointField.FLOAT32, 1),
                    PointField('ring', 20, PointField.UINT16, 1),
                ]
        velo_list = [l1[:-1] + [int(l1[-1])] for l1 in pc_velo.tolist()]
        pcl_msg = pcl2.create_cloud(header, fields, velo_list)
        self.pub.publish(pcl_msg)
        print('ouster2velo took ', (time.time() - start_time), ' seconds')
  

# MAIN FUNCTION
def main():
    rospy.init_node('inference_ros', anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--in_topic', type=str, help='from ouster', default='/os1_cloud_node1/points')
    parser.add_argument('--out_topic', type=str, help='output to be published', default='/velo/pointcloud')
    args, opts = parser.parse_known_args()

    # initialize object and open the rosbag
    obj = Ouster2Velo(args, opts)
    rospy.spin()
    
if __name__ == '__main__':
    main()