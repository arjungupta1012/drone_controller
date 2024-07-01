#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(msg):
    rospy.loginfo("Model names currently in Gazebo:")
    for model_name in msg.name:
        rospy.loginfo(f"- {model_name}")

def main():
    rospy.init_node('get_model_names_node')
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()