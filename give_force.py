import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest

def apply_force_to_rotor(link_name, force, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        req = ApplyBodyWrenchRequest()
        req.body_name = link_name
        req.reference_frame = 'world'
        req.reference_point = link_state.pose.position
        req.wrench.force.x = force[0]
        req.wrench.force.y = force[1]
        req.wrench.force.z = force[2]
        req.start_time = rospy.Time.now()
        req.duration = rospy.Duration(duration)
        apply_wrench(req)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('apply_force_example', anonymous=True)
    link_name = 'iris_base'  # Replace with your UAV rotor link name
    force = [10.0, 0.0, 0.0]  # Replace with the force vector [x, y, z]
    duration = 1.0  # Duration for which the force should be applied (in seconds)
    apply_force_to_rotor(link_name, force, duration)
