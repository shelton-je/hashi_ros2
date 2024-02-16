#!/usr/bin/env python3

"""Script to move the stewart platform to various configurations 
to OptiTrack system measurements to desired poses"""
# NOT RIGHT NOW
import json
import tqdm
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
from hashi.srv import HashiCommand, HashiCommandResponse

import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



class FTValidator(Node):

    def __init__(self):
        super().__init__('FTValidator')
        self.collected_msgs = []
        self.open_pose = [2048, 2047, 2042, 2048, 0, 0]
        self.close_pose = [2048, 1821, 2042, 2048, 0, 0]

        self.shore_hardness = self.get_parameter('~shore')
        self.pub_raw = self.create_publisher(Int32MultiArray, '/hashi/commands/raw')
        self.sub_ft = self.create_subscription(WrenchStamped, '/ft', self.ft_callback, queue_size=1)

        self.counter = 0

        for _ in range(10):
        # Close chopstick
            self.pub_raw.publish(Int32MultiArray(data=close_pose))
            self.sleep(0.5)
            # Open chopstick
            self.pub_raw.publish(Int32MultiArray(data=open_pose))
            self.sleep(0.5)
        self.dump_to_json(shore_hardness)


    def ft_callback(self, msg: WrenchStamped):
        self.collected_msgs.append({
            'time': msg.header.stamp.to_nsec(),
            'f_x': msg.wrench.force.x,
            'f_y': msg.wrench.force.y,
            'f_y': msg.wrench.force.z,
            't_x': msg.wrench.torque.x,
            't_y': msg.wrench.torque.y,
            't_z': msg.wrench.torque.z
        })

    def dump_to_json(self, shore_hardness: str):
        print(f'Saving {len(self.collected_msgs)} messages')
        with open(f'/home/river/{shore_hardness}.json', 'w') as json_file:
            json.dump(self.collected_msgs, json_file)


def main(args=None):
    rclpy.init(args=args)
    ftv = FTValidator()
    try:
        rclpy.spin(ftv)
    except KeyboardInterrupt:
        pass
    finally:
        ftv.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()