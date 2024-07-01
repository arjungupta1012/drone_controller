import rospy
from gazebo_msgs.msg import ModelStates
import math
import time
import dronekit

class DroneController:
    def __init__(self, model_name='iris_demo'):
        # Initialize attributes
        self.model_name = model_name
        self.last_errors = [0, 0, 0, 0, 0]
        self.i_errors = [0, 0, 0, 0, 0]
        self.integral = [0, 0, 0]
        self.last_errors_payload = [0, 0]
        self.i_errors_payload = [0, 0]
        self.t = 0
        self.position=[0, 0, 0]
        self.orientation=[0, 0, 0, 0]
        # Initialize ROS node and subscriber
        rospy.init_node('drone_controller_node')
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.rate = rospy.Rate(10)  # 10 Hz


####################################################################################################
##################################CALLBACKS#########################################################
####################################################################################################

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.model_name)
            model_pose = msg.pose[idx]
            self.position = [model_pose.position.x, model_pose.position.y, model_pose.position.z]
            self.orientation = [model_pose.orientation.x, model_pose.orientation.y,
                           model_pose.orientation.z, model_pose.orientation.w]
            # rospy.loginfo(f"Position of {self.model_name}: {position}")
            # rospy.loginfo(f"Orientation of {self.model_name}: {orientation}")
        except ValueError:
            rospy.logwarn(f"Model '{self.model_name}' not found in Gazebo ModelStates message")

####################################################################################################
##################################FUNCTIONS#########################################################
####################################################################################################

    def calculate_angle_error(self, current_angles, desired_angles):
        current_roll, current_pitch, current_yaw = current_angles
        desired_roll, desired_pitch, desired_yaw = desired_angles
        
        roll_error = desired_roll - current_roll
        pitch_error = desired_pitch - current_pitch
        yaw_error = desired_yaw - current_yaw
        
        return roll_error, pitch_error, yaw_error

    def calculate_angle_derivative(self, current_angles, desired_angles, dt):
        current_roll, current_pitch, current_yaw = current_angles
        desired_roll, desired_pitch, desired_yaw = desired_angles
        
        roll_error = desired_roll - current_roll
        pitch_error = desired_pitch - current_pitch
        yaw_error = desired_yaw - current_yaw
        
        roll_rate = roll_error / dt
        pitch_rate = pitch_error / dt
        yaw_rate = yaw_error / dt
        
        return roll_rate, pitch_rate, yaw_rate

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
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

    def roll_pitch_yaw(self, q):
        pitch = math.atan2(2.0*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])
        roll = math.asin(-2.0*(q[1]*q[3] - q[0]*q[2]))
        yaw = -math.atan2(2.0*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def PID(self, kp, kd, ki, error, last_error, i_err):
        out = kp * error + kd * (error - last_error) + ki * i_err
        last_error = error
        i_err += error
        return out, last_error, i_err

    def controller_drone0(self, drone, quaternions_of_drone):
        z_des = 5
        x_des = 0
        y_des = 0



    ##################### POSITION CONTROLLER ###########
    
    #####################################################
    #----------- PITCH POSITION CONTROLLER -------------#
    #####################################################
        error_y_drone = self.position[1] - y_des 
        
        kp_y_drone = 20
        kd_y_drone = 10
        ki_y_drone = 0
        
        if float(self.position[1]) != y_des:
            out_y_drone, self.last_errors[1], self.i_errors[1] = self.PID(kp_y_drone, kd_y_drone, ki_y_drone,
                                                                      error_y_drone, self.last_errors[1],
                                                                      self.i_errors[1])
            pitch_des = out_y_drone 
        else:
            pitch_des = 0



    #####################################################
    #----------- ROLL POSITION CONTROLLER -------------#
    #####################################################
        error_x_drone = x_des - float(self.position[0])
        kp_x_drone = 20
        kd_x_drone = 10
        ki_x_drone = 0
        
        if float(self.position[0]) != x_des:
            out_x_drone, self.last_errors[3], self.i_errors[3] = self.PID(kp_x_drone, kd_x_drone, ki_x_drone,
                                                                      error_x_drone, self.last_errors[3],
                                                                      self.i_errors[3])
            roll_des = out_x_drone 
        else:
            roll_des = 0



    #####################################################
    #----------- THRUST POSITION CONTROLLER -------------#
    #####################################################
        kp_thrust = 5
        kd_thrust = 2.5
        ki_thrust = 0

        # thrust_d0 = 1.962
        if float(self.position[2]) == z_des:
            thrust_d0 = 1.962
        else:
            error_z_drone = z_des - float(self.position[2])
            force = thrust_d0 + kp_thrust * error_z_drone + kd_thrust * error_z_drone / 0.01 + ki_thrust * (i_errort_d0)
            last_error_d0 = error_z_drone
            i_error+=error_z_drone

            thrust_d0 = force/4.0


        # last_errorr_d0 = roll_des - roll_drone0
        i_error = [self.i_errors[3], self.i_errors[1], i_errort_d0]
        return control_d0

    def controller_drone1(self, target_roll, target_pitch, target_yaw, current_position, t):
        current_angles = current_position[0], current_position[1], current_position[2]
        desired_angles = target_roll, target_pitch, target_yaw
        roll_rate, pitch_rate, yaw_rate = self.calculate_angle_derivative(current_angles, desired_angles, t)
        # Add your control logic here

    def run(self):
        start = time.time()
        while not rospy.is_shutdown() and time.time() - start < 180:
            step_start = time.time()

            # Implement your control logic here

            self.rate.sleep()

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
