import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Empty

class DroneController(Node):
    def __init__(self, model_name='iris_demo'):
        super().__init__('drone_controller_node')
        self.model_name = model_name
        self.last_errors = [0, 0, 0]
        self.i_errors = [0, 0, 0]
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]

        # Desired x, y positions during takeoff
        self.x_des = 0.0
        self.y_des = 0.0
        self.z_des = 5.0

        # Subscribe to the ground truth pose topic
        self.subscription = self.create_subscription(Pose, '/drone/gt_pose', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.takeoff_publisher = self.create_publisher(Empty, '/drone/takeoff', 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # State to manage takeoff
        self.takeoff_done = False

        # Drone PID parameters
        self.pid_params = {
            'roll_pitch': {'kp': 5, 'kd': 1, 'ki': 0.2, 'limit': 0.5},  # Adjusted PID values
            'yaw': {'kp': 1, 'kd': 0.5, 'ki': 0.1, 'limit': 1.5},
            'velocity_x': {'kp': 3, 'kd': 1, 'ki': 0.1, 'limit': 2},
            'velocity_y': {'kp': 3, 'kd': 1, 'ki': 0.1, 'limit': 2},
            'velocity_z': {'kp': 3, 'kd': 1, 'ki': 0.1, 'limit': 1},
            'position_xy': {'kp': 0.5, 'kd': 0.2, 'ki': 0, 'limit': 5},
            'position_z': {'kp': 1, 'kd': 0.2, 'ki': 0, 'limit': 1},
        }

    def pose_callback(self, msg):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.get_logger().info(f"Position updated: {self.position}")

    def PID(self, kp, kd, ki, error, last_error, i_err, dt, limit):
        p_term = kp * error
        d_term = kd * (error - last_error) / dt
        i_err += error * dt
        i_term = ki * i_err

        output = p_term + d_term + i_term
        output = max(min(output, limit), -limit)
        return output, error, i_err

    def control_drone(self, x_des, y_des, z_des, dt):
        error_x = x_des - self.position[0]
        error_y = y_des - self.position[1]
        error_z = z_des - self.position[2]

        control_x, self.last_errors[0], self.i_errors[0] = self.PID(
            self.pid_params['velocity_x']['kp'],
            self.pid_params['velocity_x']['kd'],
            self.pid_params['velocity_x']['ki'],
            error_x, self.last_errors[0], self.i_errors[0], dt,
            self.pid_params['velocity_x']['limit']
        )

        control_y, self.last_errors[1], self.i_errors[1] = self.PID(
            self.pid_params['velocity_y']['kp'],
            self.pid_params['velocity_y']['kd'],
            self.pid_params['velocity_y']['ki'],
            error_y, self.last_errors[1], self.i_errors[1], dt,
            self.pid_params['velocity_y']['limit']
        )

        control_z, self.last_errors[2], self.i_errors[2] = self.PID(
            self.pid_params['position_z']['kp'],
            self.pid_params['position_z']['kd'],
            self.pid_params['position_z']['ki'],
            error_z, self.last_errors[2], self.i_errors[2], dt,
            self.pid_params['position_z']['limit']
        )

        cmd = Twist()
        cmd.linear.x = float(control_x)
        cmd.linear.y = float(control_y)
        cmd.linear.z = float(control_z)
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.get_logger().info(f"Control X: {control_x}, Control Y: {control_y}, Control Z: {control_z}")
        self.get_logger().info(f"Errors X: {error_x}, Y: {error_y}, Z: {error_z}")

        return cmd

    def control_loop(self):
        dt = 0.1

        if not self.takeoff_done:
            if self.position[2] < self.z_des - 0.5:
                self.get_logger().info(f"Waiting for altitude: {self.position[2]:.2f} / {self.z_des:.2f}")
                # Set desired positions to current position during takeoff phase
                self.x_des = self.position[0]
                self.y_des = self.position[1]
            else:
                self.takeoff_done = True
                self.get_logger().info(f"Takeoff completed. Starting altitude stabilization.")

        cmd = self.control_drone(self.x_des, self.y_des, self.z_des, dt)
        self.cmd_vel_publisher.publish(cmd)
              
def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()

    controller.takeoff_publisher.publish(Empty())
    controller.get_logger().info("Takeoff command sent. Waiting for altitude stabilization.")

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
