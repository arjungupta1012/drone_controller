from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import time,math
# Constants for PID control
KP = 0.5  # Proportional gain
KI = 0.1  # Integral gain
KD = 0.2  # Derivative gain

# Constants for the UAV dynamics
MASS = 1.0  # Mass of the UAV
MAX_THRUST = 1.0  # Maximum thrust that the UAV can produce

# Target position (desired position to hold)
target_position = (0.0, 0.0, 2.0)  # Format: (lat, lon, alt)

# Connect to the UAV
connection_string = 'udp:127.0.0.1:14550'  # Example connection string, replace with the correct one
vehicle = connect(connection_string, wait_ready=True)

# # Arm and takeoff the UAV
# vehicle.mode = VehicleMode("GUIDED")
# vehicle.armed = True
# vehicle.simple_takeoff(target_position[2])

# # Wait until the UAV reaches the desired altitude
# while True:
#     if vehicle.location.global_relative_frame.alt >= target_position[2] * 0.95:
#         print("Reached target altitude.")
#         break
#     time.sleep(1)

# Variables for PID control


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
        current_altitude = vehicle.location.local_frame[2]
        print (" Altitude: "), current_altitude
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print ("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

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

def land_no_gps():
    print("Descending and landing")
    #while vehicle.location.global_relative_frame.alt > 0.1:
    while  vehicle.location.local_frame[2]> 0.1:
        send_local_ned_velocity(0, 0, -0.5)  # Adjust the descent rate as per your requirements
        time.sleep(1)

    # Disarm the UAV
    vehicle.armed = False
    return

arm_and_takeoff_nogps(target_position[2])

integral = (0.0, 0.0, 0.0)  # Integral term
previous_error = (0.0, 0.0, 0.0)  # Previous error term

# Time variables
start_time = time.time()
previous_time = start_time

while True:
    current_time = time.time()
    dt = current_time - previous_time

    # Get the current position of the UAV in GPS coordinates
    current_position = (
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt,
    )

    # Calculate the error in position
    error = (
        target_position[0] - current_position[0],
        target_position[1] - current_position[1],
        target_position[2] - current_position[2],
    )

    # Calculate the control signals using PID
    control_signals = (
        KP * error[0] + KI * integral[0] + KD * (error[0] - previous_error[0]),
        KP * error[1] + KI * integral[1] + KD * (error[1] - previous_error[1]),
        KP * error[2] + KI * integral[2] + KD * (error[2] - previous_error[2]),
    )

    # Update the integral term
    integral = (
        integral[0] + error[0] * dt,
        integral[1] + error[1] * dt,
        integral[2] + error[2] * dt,
    )

    # Update the previous error term
    previous_error = error

    # Calculate the thrust commands based on control signals
    thrust_commands = (
        MAX_THRUST + control_signals[0] / MASS,
        MAX_THRUST + control_signals[1] / MASS,
        MAX_THRUST + control_signals[2] / MASS,
    )

    # Set the UAV velocity in the body frame
    vehicle.velocity = (thrust_commands[0], thrust_commands[1], thrust_commands[2])

    # Print the current position
    print(f"Position: {current_position}")

    # Check if position is within a certain tolerance
    position_tolerance = 0.0001  # Tolerance for position hold (in degrees)
    if (
        abs(error[0]) <= position_tolerance
        and abs(error[1]) <= position_tolerance
        and abs(error[2]) <= position_tolerance
    ):
        print("Position hold achieved.")
        break

    previous_time = current_time

# # Land the UAV and disarm it
# vehicle.mode = VehicleMode("LAND")
# while vehicle.armed:
#     time.sleep(1)

# # Close the connection to the UAV
# vehicle.close()
land_no_gps()