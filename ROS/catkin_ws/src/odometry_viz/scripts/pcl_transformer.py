import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs
import tf2_py as tf2

def callback(msg):
    try:
        # Look up the transformation
        trans = tf_buffer.lookup_transform('/camera_init_rot', msg.header.frame_id, rospy.Time())
        # Apply the transformation
        msg_rotated = tf2_sensor_msgs.do_transform_cloud(msg, trans)
        # Now msg_rotated is the input message, but rotated 90 degrees
        # Do something with msg_rotated...
    except (tf2.LookupException, tf2.ExtrapolationException) as e:
        rospy.logerr("Transform error: %s", e)

rospy.init_node('my_node')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
rospy.Subscriber('input_topic', PointCloud2, callback)
rospy.spin()