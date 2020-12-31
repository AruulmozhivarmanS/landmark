import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


if __name__=="__main__":
    rospy.init_node("correct_error", anonymous=True)
    buf = tf.Buffer()
    ls = tf.TransformListener(buf)
    br = tf.TransformBroadcaster()

    try:
        while not rospy.is_shutdown():
            try:
                check_tf = buf.lookup_transform('odom_correct', 'tag_world_cur', rospy.Time.now(), rospy.Duration(1.0))
                cur_tf = buf.lookup_transform('odom', 'tag_world_cur', rospy.Time.now(), rospy.Duration(1.0))
                ref_tf = buf.lookup_transform('odom', 'tag_world_ref', rospy.Time.now(), rospy.Duration(1.0))
                print(check_tf)
                print(ref_tf)
                print(cur_tf)
            except:
                continue

    except KeyboardInterrupt:
        exit(0)

