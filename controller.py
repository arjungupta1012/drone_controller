#!/usr/bin/env python
import time
import dronekit
import math
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

#################################################################################################
######################CALLBACKS##################################################################
#################################################################################################
def model_states_callback(msg):
    model_name = 'iris_demo'  # Replace with the name of your model
    try:
        idx = msg.name.index(model_name)
        model_pose = msg.pose[idx]
        position = ["{:.3f}".format(model_pose.position.x), "{:.3f}".format(model_pose.position.y), "{:.3f}".format(model_pose.position.z)]
        orientation = ["{:.3f}".format(model_pose.orientation.x), "{:.3f}".format(model_pose.orientation.y), "{:.3f}".format(model_pose.orientation.z), "{:.3f}".format(model_pose.orientation.w)]
        # rospy.loginfo(f"Position of {model_name}: ({model_pose.position.x}, {model_pose.position.y}, {model_pose.position.z})")
        # rospy.loginfo(f"Orientation of {model_name}: ({model_pose.orientation.x}, {model_pose.orientation.y}, {model_pose.orientation.z}, {model_pose.orientation.w})")
        print(f"Position of {model_name}: {position}")
        print(f"Orientation of {model_name}: {orientation}")
    except ValueError:
        rospy.logwarn(f"Model '{model_name}' not found in Gazebo ModelStates message")



def calculate_angle_derivative(current_angles, desired_angles, dt):
    """
    Calculate the derivative (angular velocity) of roll, pitch, and yaw angles.

    Parameters:
    - current_angles: Tuple or list of current roll, pitch, and yaw angles in radians (roll, pitch, yaw).
    - desired_angles: Tuple or list of desired roll, pitch, and yaw angles in radians (roll, pitch, yaw).
    - dt: Time step (in seconds) between the current and desired angles.

    Returns:
    - Tuple containing the derivative of roll, pitch, and yaw angles (roll_rate, pitch_rate, yaw_rate).
    """
    # Extract current and desired angles
    current_roll, current_pitch, current_yaw = current_angles
    desired_roll, desired_pitch, desired_yaw = desired_angles
    
    # Calculate angular differences (error)
    roll_error = desired_roll - current_roll
    pitch_error = desired_pitch - current_pitch
    yaw_error = desired_yaw - current_yaw
    
    # Calculate angular rates (derivative)
    roll_rate = roll_error / dt
    pitch_rate = pitch_error / dt
    yaw_rate = yaw_error / dt
    
    return roll_rate, pitch_rate, yaw_rate

def calculate_angle_derivative(current_angles, desired_angles,dt):
        # Extract current and desired angles
        current_roll, current_pitch, current_yaw = current_angles
        desired_roll, desired_pitch, desired_yaw = desired_angles
        
        # Calculate errors
        roll_error = desired_roll - current_roll
        pitch_error = desired_pitch - current_pitch
        yaw_error = desired_yaw - current_yaw


        Kp=1
        Ki=0.01
        
        # Update integral of errors
        integral[0] += roll_error * dt
        integral[1] += pitch_error * dt
        integral[2] += yaw_error * dt
        
        # Calculate control outputs (PI control)
        roll_control  = Kp * roll_error  + Ki * integral[0]
        pitch_control = Kp * pitch_error + Ki * integral[1]
        yaw_control   = Kp * yaw_error   + Ki * integral[2]
        
        return roll_control, pitch_control, yaw_control


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """Convert degrees to quaternions."""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    t0 = math.cos(-yaw * 0.5)
    t1 = math.sin(-yaw * 0.5)
    t2 = math.cos(pitch * 0.5)
    t3 = math.sin(pitch * 0.5)
    t4 = math.cos(roll * 0.5)
    t5 = math.sin(roll * 0.5)
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]

def roll_pitch_yaw(q):
    pitch = math.atan2(2.0*(q[1]*q[2] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])
    roll = math.asin(-2.0*(q[1]*q[3] - q[0]*q[2]))
    yaw = - math.atan2(2.0*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def PID(kp,kd,ki,error,last_error,i_err):
    out = kp*error + kd*(error-last_error) + ki*i_err
    last_error = error
    i_err += error
    return out, last_error, i_err

def controller_drone0(self,last_errors_d0[0],last_errors_d0[1],last_errors_d0[2],last_errors_d0[3],last_errors_d0[4],t,i_errors_d0[0],i_errors_d0[1],i_errors_d0[2],i_errors_d0[3],i_errors_d0[4],last_errors_payload[0],last_errors_payload[1],i_errors_payload[0],i_errors_payload[1]):
  
    z_des = 5
    x_des = 0 
    y_des = 0

    ############# POSITION CONTROLLER ###################
    
    #####################################################
    #----------- PITCH POSITION CONTROLLER -------------#
    #####################################################
    
    y_current_drone = float(self.position[1])

    if y_current_drone!= y_des:
       
       error_y_drone = -y_des + y_current_drone

       kp_y_drone = 20
       kd_y_drone = 10
       ki_y_drone = 0

       out_y_drone, last_error_y_drone, i_error_y_drone = PID(kp_y_drone,kd_y_drone,ki_y_drone,
                                                 error_y_drone,last_error_y_drone,i_error_y_drone)
        
       pitch_des = out_y_drone

    else:
       pitch_des = 0


    #####################################################
    #----------- ROLL POSITION CONTROLLER --------------#
    #####################################################
    
    x_current_drone = float(self.position[0])
    
    if x_current_drone != x_des:
        error_x_drone = x_des - x_current_drone
        
        kp_x_drone = 20
        kd_x_drone = 10
        ki_x_drone = 0
        
        
        out_x_drone, last_error_x_drone, i_error_x_drone = PID(kp_x_drone,kd_x_drone,ki_x_drone,
                                                 error_x_drone,last_error_x_drone,i_error_x_drone)
        
        roll_des = out_x_drone 
        
    else:
        roll_des = 0



    ############ CURRENT POSITIONS #####################
    print("CURRENT DRONE POS: ",self.position)

    roll_drone0=roll_pitch_yaw(self.orientation)[0]
    pitch_drone0=roll_pitch_yaw(self.orientation)[1]

    ### When mass = 0.1kg
    ### When thrust = 0.981 drone hovers
    
    #-----------------------------------------#
    ########### THRUST CONTROLLER #############
    #-----------------------------------------#
    if float(self.position[2]) == z_des:
        thrust_d0 = 1.962
        last_errort_d0 = 0
        
    else:

        kp = 5
        kd = 2.5
        ki = 0
        errort_d0 = (z_des - float(self.position[2]))
        force = 1.962 + kp*errort_d0 + kd*(errort_d0-last_errort_d0)/0.01 + ki*(i_errort_d0)
        last_errort_d0 = errort_d0    
        i_errort_d0 += errort_d0      
        # print("FORCE: ",force)
        thrust_d0 = (force)/4
    
    if pitch_drone0 != pitch_des:
        errorp_d0 = pitch_des - pitch_drone0
        kp_pitch = 5
        kd_pitch = 2.5
        ki_pitch = 0
        #print("ERROR PITCH: ", errorp_d0)
        # print("LAST ERROR PITCH: ", last_errorp_d0)
        pitch_d0 = kp_pitch*errorp_d0 + kd_pitch*(errorp_d0 - last_errorp_d0) + ki_pitch*(i_errorp_d0)
        last_errorp_d0 = errorp_d0
        i_errorp_d0 += errorp_d0
    else:
        pitch_d0 = 0
    
    #-----------------------------------------#
    ########### ROLL CONTROLLER ##############
    #-----------------------------------------#
    
    if roll_drone0 != roll_des:
        errorr_d0 = roll_des - roll_drone0
        kp_roll = 5
        kd_roll = 2.5
        ki_roll = 0
        #print("ERROR ROLL: ", errorr_d0)
        roll_d0 = kp_roll*errorr_d0 + kd_roll*(errorr_d0 - last_errorr_d0) + ki_roll*(i_errorr_d0)*0.01
        last_errorr_d0 = errorr_d0
        i_errorr_d0 += errorr_d0
    else:
        roll_d0 = 0
    

    control_d0 = [thrust_d0,pitch_d0,roll_d0]
    last_errors_d0 = [last_errort_d0,last_errorp_d0,last_errorr_d0,last_error_x_drone,last_error_y_drone]
    i_error = [i_errort_d0,i_errorp_d0,i_errorr_d0,i_error_x_drone,i_error_y_drone]

    return control_d0, last_errors_d0, t, i_error

def controller_drone1(target_roll,target_pitch,target_yaw,drone.pos[0],drone.pos[1],drone.pos[2],t):
    current_angles = drone.pos[0],drone.pos[1],drone.pos[2]
    desired_angles = target_roll,target_pitch,target_yaw
    roll_rate,pitch_rate,yaw_rate = calculate_angle_derivative(current_angles, desired_angles, t)

    




def main():
    rospy.init_node('get_model_names_node')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    rospy.spin()

if __name__ == '__main__':
    main()