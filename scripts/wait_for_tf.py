import tf2_ros
from geometry_msgs.msg import TransformStamped

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

try:
    # Wait up to 1.0 second for the transform
    transform = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    rospy.logwarn("Failed to lookup transform: %s", str(e))
    return