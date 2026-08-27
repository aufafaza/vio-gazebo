#!/usr/bin/env python3
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').get_parameter_value().string_value

        with open(path) as f:
            config = yaml.safe_load(f)

        br = StaticTransformBroadcaster(self)
        transforms = []

        for t in config['static_transforms']:
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id  = t['parent_frame']
            msg.child_frame_id   = t['child_frame']
            msg.transform.translation.x = t['translation']['x']
            msg.transform.translation.y = t['translation']['y']
            msg.transform.translation.z = t['translation']['z']
            msg.transform.rotation.x = t['rotation']['x']
            msg.transform.rotation.y = t['rotation']['y']
            msg.transform.rotation.z = t['rotation']['z']
            msg.transform.rotation.w = t['rotation']['w']
            transforms.append(msg)

        br.sendTransform(transforms)
        self.get_logger().info(f'Published {len(transforms)} static transforms from {path}')

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
