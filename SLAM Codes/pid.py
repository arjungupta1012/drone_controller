from dronekit import connect, LocationLocal
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time

# Connect to the Pixhawk Cube
connection_string = '127.0.0.1:14555'  # Replace with the connection string for your specific setup
vehicle = connect(connection_string, wait_ready=False, baud=57600)

# Callback function to receive Odometry messages
# file1 = open("/home/uasdtu/Desktop/indoor/logs.txt","a")
def odom_callback(odom_msg):
    # odom_send = odom_msg
    global position_x,position_y,position_z,orientataion_w,orientataion_y,orientataion_z,orientataion_x,timestamps
    # Extract position information from the Odometry message
    position_x = odom_msg.pose.pose.position.x
    position_y = odom_msg.pose.pose.position.y
    position_z = odom_msg.pose.pose.position.z
    orientataion_x=odom_msg.pose.pose.orientation.x
    orientataion_y=odom_msg.pose.pose.orientation.y
    orientataion_z=odom_msg.pose.pose.orientation.z
    orientataion_w=odom_msg.pose.pose.orientation.w
    timestamp=odom_msg.header.stamp


    pose = PoseStamped()
    # Set the position and orientation based on your vision system's output
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = position_z
    pose.pose.orientation.x = orientataion_x
    pose.pose.orientation.y = orientataion_y
    pose.pose.orientation.z = orientataion_z
    pose.pose.orientation.w = orientataion_w

    pub.publish(pose)

    # print("recived ROS odometry messages")
    # print("position x : =%.2f,y=%.2f,z=%.2f" % (position_x,position_y,position_z))
    # print("orientataion x : =%.2f,y=%.2f,z=%.2f,w=%.2f" % (orientataion_x,orientataion_y,orientataion_z,orientataion_w))
    # print("timestamp=%s",timestamp)

    # Update the current position of the vehicle
    # odom_pub.publish(odom_send)

    # desired_location = LocationLocal(north=position_x, east=position_y, down=position_z)  # Replace with your desired coordinates

    # Set the vehicle's location to the desired location
    # vehicle.location.local_relative_frame = desired_location
    # print(vehicle.location.local_relative_frame)

    #vehicle.location.local_frame = (position_x, position_y, position_z)
# Subscribe to the Odometry topic
rospy.init_node('odom_listener2')
odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)

pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)


# Wait for the first Odometry message to arrive
while not rospy.is_shutdown():
    #if vehicle.location.local_frame is not None:

        
    time.sleep(1)
