from dronekit import connect, LocationLocal
import rospy
from nav_msgs.msg import Odometry
import time
from takoff2 import DroneController

connection_string = '127.0.0.1:14552'  # Replace with the connection string for your specific setup
vehicle = connect(connection_string, wait_ready=True, baud=57600)

class OdomListener:
    def __init__(self):
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientataion_w = 0.0
        self.orientataion_y = 0.0
        self.orientataion_z = 0.0
        self.orientataion_x = 0.0
        self.timestamps = None

        rospy.init_node('odom_listener')
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

    def odom_callback(self, odom_msg):
        self.position_x = odom_msg.pose.pose.position.x
        self.position_y = odom_msg.pose.pose.position.y
        self.position_z = odom_msg.pose.pose.position.z
        self.orientataion_x = odom_msg.pose.pose.orientation.x
        self.orientataion_y = odom_msg.pose.pose.orientation.y
        self.orientataion_z = odom_msg.pose.pose.orientation.z
        self.orientataion_w = odom_msg.pose.pose.orientation.w
        self.timestamps = odom_msg.header.stamp

        print("Received ROS odometry messages")
        print("Position: x=%.2f, y=%.2f, z=%.2f" % (self.position_x, self.position_y, self.position_z))
        print("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f" % (self.orientataion_x, self.orientataion_y, self.orientataion_z, self.orientataion_w))
        print("Timestamp: %s" % self.timestamps)

# Create an instance of OdomListener
odom_listener = OdomListener()

# Access the position variables using the instance
print(odom_listener.position_x)
print(odom_listener.position_y)
print(odom_listener.position_z)

# Use the variables in Code 1 by updating the local_frame in the DroneController class
drone_controller = DroneController()
desired_location = LocationLocal(north=odom_listener.position_x, east=odom_listener.position_y, down=odom_listener.position_z)
drone_controller.vehicle.location.local_relative_frame = desired_location
