import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_publisher')

    # Create a tf broadcaster
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        # Set the fixed identity transformation between "camera_link" and "base_link"
        translation = (0.0, 0.0, 0.0)  # Zero translation
        rotation = (0.0, 0.0, 0.0, 1.0)  # Identity rotation (quaternion)

        # Publish the fixed transformation from "camera_link" to "base_link"
        broadcaster.sendTransform(translation,
                                  rotation,
                                  rospy.Time.now(),
                                  "base_link",
                                  "camera_link")

        # Sleep for a small duration
        rospy.sleep(0.1)