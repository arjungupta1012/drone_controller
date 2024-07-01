#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def model_state_callback(msg):
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

def main():
    rospy.init_node('get_model_coordinates_node')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
