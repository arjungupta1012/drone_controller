from dronekit import connect, LocationLocal
import rospy
from nav_msgs.msg import Odometry
import time

# Connect to the Pixhawk Cube
connection_string = '127.0.0.1:14552'  # Replace with the connection string for your specific setup
vehicle = connect(connection_string, wait_ready=False, baud=57600)

# Callback function to receive Odometry messages
# file1 = open("/home/uasdtu/Desktop/indoor/logs.txt","a")
def odom_callback(odom_msg):
    odom_send = odom_msg
    global position_x,position_y,position_z,orientataion_w,orientataion_y,orientataion_z,orientataion_x,timestamps
    # Extract position information from the Odometry message

    # child_frame=odom_send.child_frame_id
    position_x = odom_msg.pose.pose.position.x
    position_y = odom_msg.pose.pose.position.y
    position_z = odom_msg.pose.pose.position.z
    orientataion_x=odom_msg.pose.pose.orientation.x
    orientataion_y=odom_msg.pose.pose.orientation.y
    orientataion_z=odom_msg.pose.pose.orientation.z
    orientataion_w=odom_msg.pose.pose.orientation.w
    timestamp=odom_msg.header.stamp



    print("recived ROS odometry messages")
    print("child")
    print("position x : =%.2f,y=%.2f,z=%.2f" % (position_x,position_y,position_z))
    print("orientataion x : =%.2f,y=%.2f,z=%.2f,w=%.2f" % (orientataion_x,orientataion_y,orientataion_z,orientataion_w))
    print("timestamp=%s",timestamp)

    # Update the current position of the vehicle
    odom_pub.publish(odom_send)

    desired_location = LocationLocal(north=position_x, east=position_y, down=position_z)  # Replace with your desired coordinates

    # Set the vehicle's location to the desired location
    vehicle.location.local_relative_frame = desired_location
    print(vehicle.location.local_relative_frame)

    #vehicle.location.local_frame = (position_x, position_y, position_z)
# Subscribe to the Odometry topic
rospy.init_node('odom_listener')
odom_sub = rospy.Subscriber('/rtabmap/odom', Odometry, odom_callback)
odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry,queue_size=10)


# Wait for the first Odometry message to arrive
while not rospy.is_shutdown():
    # file1.write("position x : =%.2f,y=%.2f,z=%.2f" % (position_x,position_y,position_z))
    # file1.write("orientataion x : =%.2f,y=%.2f,z=%.2f,w=%.2f" % (orientataion_x,orientataion_y,orientataion_z,orientataion_w))
    # file1.write("timestamp=%s",timestamp)
    #if vehicle.location.local_frame is not None:

        
    time.sleep(1)
