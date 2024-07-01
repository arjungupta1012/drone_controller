import rospy
from gazebo_msgs.msg import ModelStates
def model_states_callback(data):
    try:
        # Find the index of your model in the model list
        idx = data.name.index("iris_demo")
        
        # Access the linear and angular velocities of your model
        linear_velocity = data.twist[idx].linear
        angular_velocity = data.twist[idx].angular
        
        # rospy.loginfo("Linear Velocity of your_model_name: %s", linear_velocity)
        rospy.loginfo("Angular Velocity of your_model_name: %s", angular_velocity)
        
    except ValueError:
        rospy.logwarn("Model 'your_model_name' not found in model_states message")

def listener():
    rospy.init_node('model_velocity_listener', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()