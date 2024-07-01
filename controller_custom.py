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
        self.integral_error = [0, 0, 0]
        self.integral_error_rate = [0, 0, 0]
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
            self.linear_velocity = msg.twist[idx].linear
            self.angular_velocity = msg.twist[idx].angular
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

    def angle_PI_controller(self, current_angles, desired_angles, integral_error, dt):
        current_roll, current_pitch, current_yaw = current_angles
        desired_roll, desired_pitch, desired_yaw = desired_angles
        
        roll_error = desired_roll - current_roll
        pitch_error = desired_pitch - current_pitch
        yaw_error = desired_yaw - current_yaw
        #################################################################################################################
        Kp_roll = 0.5
        Ki_roll = 0

        roll_rate_desired = Kp_roll * roll_error + Ki_roll * integral_error[0] * dt
        integral_error[0] = integral_error[0] + roll_error
        #################################################################################################################
        Kp_pitch = 0.5
        Ki_pitch = 0

        pitch_rate_desired = Kp_pitch * pitch_error + Ki_pitch * integral_error[1] * dt
        integral_error[1] = integral_error[1] + pitch_error
        #################################################################################################################     
        Kp_yaw = 0.5
        Ki_yaw = 0

        yaw_rate_desired = Kp_yaw * yaw_error + Ki_yaw * integral_error[2] * dt
        integral_error[2] = integral_error[2] + yaw_error
        #################################################################################################################
        return roll_rate_desired, pitch_rate_desired, yaw_rate_desired

    def angular_rate_PI_controller(self, current_angular_rate, roll_rate_desired, pitch_rate_desired, yaw_rate_desired, integral_error_rate, dt):
        current_roll_rate, current_pitch_rate, current_yaw_rate = current_angular_rate

        Kp_roll_rate = 0.5
        Ki_roll_rate = 0

        roll_rate_error = roll_rate_desired - current_roll_rate
        U2 = Kp_roll_rate * roll_rate_error + Ki_roll_rate * integral_error_rate[0] * dt
        integral_error_rate[0] = integral_error_rate[0] + roll_rate_error



        Kp_pitch_rate = 0.5
        Ki_pitch_rate = 0

        pitch_rate_error = pitch_rate_desired - current_pitch_rate
        U3 = Kp_pitch_rate * pitch_rate_error + Ki_pitch_rate * integral_error_rate[1] * dt
        integral_error_rate[1] = integral_error_rate[1] + pitch_rate_error



        Kp_yaw_rate = 0.5
        Ki_yaw_rate = 0

        yaw_rate_error = yaw_rate_desired - current_yaw_rate
        U4 = Kp_yaw_rate * yaw_rate_error + Ki_yaw_rate * integral_error_rate[2] * dt
        integral_error_rate[2] = integral_error_rate[2] + yaw_rate_error

        
        return U2, U3, U4
   
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

    def controller_drone1(self, desired_angles, current_position, current_angular_rate, integral_error, integral_error_rate, dt):
        
        roll_rate_desired, pitch_rate_desired, yaw_rate_desired = self.angle_PI_controller(current_position, desired_angles, integral_error, dt)

        U2, U3, U4 = self.angular_rate_PI_controller(current_angular_rate, roll_rate_desired, pitch_rate_desired, yaw_rate_desired, integral_error_rate, dt)

        # Add your control logic here

    def run(self):
        start = time.time()
        while not rospy.is_shutdown() and time.time() - start < 180:
            step_start = time.time()

            desired_roll = 0
            desired_pitch = 0
            desired_yaw = 0

            desired_angles = [desired_roll, desired_pitch, desired_yaw]
            current_angles = self.roll_pitch_yaw(self.orientation)
            current_angular_rate = self.angular_velocity
            self.controller_drone1(desired_angles, current_angles, current_angular_rate, time.time() - step_start)

            self.rate.sleep()

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
