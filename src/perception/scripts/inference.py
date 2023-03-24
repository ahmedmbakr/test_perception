#!/usr/bin/env python3

import rospy
from rospy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import perception_lidar.point_cloud2
from spvnas.model_zoo import spvnas_specialized


import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscription = self.create_subscription(PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)

    def pointcloud_callback(self, msg):
        point_count = len(msg.data)
        arr = np.empty((point_count, 5))
        for i, el in enumerate(perception_lidar.point_cloud2.read_points(msg)):
            arr[i] = el

        self.get_logger().info("%d" % len(msg.data))
        # arr = np.fromiter(motor_drivers.point_cloud2.read_points(msg), float, count=-1)
        # np.set_printoptions(threshold=np.inf)
        print(arr.shape, arr[:, :4].shape)
        print(arr, arr[:, :4])


def main(args=None):
    rospy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rospy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rospy.shutdown()


if __name__ == '__main__':
    main()
print("Hello World")
