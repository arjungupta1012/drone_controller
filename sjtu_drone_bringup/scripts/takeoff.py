# import rclpy
# from std_msgs.msg import Empty

# def main(args=None):
#     rclpy.init(args=args)

#     node = rclpy.create_node('takeoff_publisher')

#     # Create a publisher
#     publisher = node.create_publisher(Empty, '/drone/takeoff', qos_profile=rclpy.qos.qos_profile_sensor_data)

#     # Create an Empty message instance
#     msg = Empty()

#     # Publish the message once
#     publisher.publish(msg)

#     # Spin briefly to allow the message to be published
#     rclpy.spin_once(node, timeout_sec=1.0)

#     # Shutdown the node and destroy the publisher explicitly
#     node.destroy_publisher(publisher)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from std_msgs.msg import Empty

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('takeoff_publisher')

    # Create a publisher on the /drone/takeoff topic
    publisher = node.create_publisher(Empty, '/drone/takeoff', 10)

    # Create an instance of the message type
    msg = Empty()

    # Publish the message once
    publisher.publish(msg)

    node.get_logger().info('Takeoff command published')

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
