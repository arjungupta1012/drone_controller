from dronekit import connect, VehicleMode
from pymavlink import mavutil
#import rospy
import math
#from nav_msgs.msg import Odometry
import time

# Connect to the Pixhawk Cube
connection_string = '127.0.0.1:14455'
# Replace with the connection string for your specific setup
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Callback function to receive Odometry messages
#def odom_callback(odom_msg):
#    # Extract position information from the Odometry message
#    position_x = odom_msg.pose.pose.position.x
#    position_y = odom_msg.pose.pose.position.y
#    position_z = odom_msg.pose.pose.position.z
#
#    # Update the current position of the vehicle
#    vehicle.location.local_frame = (position_x, position_y, position_z)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version    
   (sending the message multiple times does not cause problems).
        
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(                                                             0,
                                                             0,                                         #target system
                                                             0,                                         #target component
                                                             0b00000000,                                #type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),    #q
                                                             0,                                         #body roll rate in radian
                                                             0,                                         #body pitch rate in radian
                                                             math.radians(yaw_rate),                    #body yaw rate in radian
                                                             thrust)                                    #thrust
    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle

        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print ("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)


    print ("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print (" Altitude: "), current_altitude
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print ("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

def land_no_gps():
    print("Descending and landing")
    #while vehicle.location.global_relative_frame.alt > 0.1:
    while  vehicle.location.local_frame[2]> 0.1:
        send_local_ned_velocity(0, 0, -0.5)  # Adjust the descent rate as per your requirements
        time.sleep(1)

    # Disarm the UAV
    vehicle.armed = False
    return

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


## Subscribe to the Odometry topic
#rospy.init_node('odom_listener')
#odom_sub = rospy.Subscriber('odom_topic', Odometry, odom_callback)
#
## Wait for the first Odometry message to arrive
#while not rospy.is_shutdown():
#    if vehicle.location.local_frame is not None:
#        break
#    time.sleep(1)
#
## Function to arm the UAV


# Define the target altitude (in meters) for takeoff
target_altitude = 2

# Arm and takeoff to the target altitude
arm_and_takeoff_nogps(target_altitude)

# Move the UAV by sending local position commands
target_position = [2, 0, 2]  # Replace with your target position [north, east, down] relative to the takeoff point

print("Moving to target position")
send_local_ned_velocity(target_position[0], target_position[1], target_position[2])

# Wait for the UAV to reach the target position
while True:
    current_position = vehicle.location.local_frame
    distance = abs(current_position.north - target_position[0]) + abs(current_position.east - target_position[1]) + abs(current_position.down - target_position[2])
    print("Distance to target position: %.2f meters" % distance)

    if distance <= 0.5:
        print("Target position reached")
        break

    time.sleep(1)


# Land the UAV by descending in GUIDED mode
land_no_gps()
# Close the connection
vehicle.close()

