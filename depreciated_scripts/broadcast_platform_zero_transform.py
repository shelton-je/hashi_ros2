#!/usr/bin/env python3
# NOT RIGHT NOW
import rclpy
from rclpy.node import Node

import tf2_ros
import geometry_msgs.msg

class StaticTf2Broadcaster(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')


        self.declare_parameter('zero_translation', 0.0) 
        self.declare_parameter('zero_rotation', 0.0) 
        self.set_parameters([rclpy.parameter.Parameter('zero_translation', rclpy.Parameter.Type.DOUBLE, 0.0), 
                             rclpy.parameter.Parameter('zero_rotation', rclpy.Parameter.Type.DOUBLE, 0.0)])




        # these values are set by running zero_platform_and_record_origin_tf.py
        trans = self.get_parameter('zero_translation')
        rot = self.get_parameter('zero_rotation')
        
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = "hashi_world"
        static_transformStamped.child_frame_id = "hashi_zero"

        static_transformStamped.transform.translation.x = trans[0]
        static_transformStamped.transform.translation.y = trans[1]
        static_transformStamped.transform.translation.z = trans[2]

    
        static_transformStamped.transform.rotation.x = rot[0]
        static_transformStamped.transform.rotation.y = rot[1]
        static_transformStamped.transform.rotation.z = rot[2]
        static_transformStamped.transform.rotation.w = rot[3]
        broadcaster.sendTransform(static_transformStamped)


def main(args=None):
    rclpy.init(args=args)
    static_tf_broadcaster = StaticTf2Broadcaster()
    try:
        rclpy.spin(static_tf_broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        static_tf_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

