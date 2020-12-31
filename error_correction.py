import rospy
import tf2_ros as tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

def find_error(ref, cur):
    err = TransformStamped()
    err.header = cur.header

    err.transform.translation.x = ref.transform.translation.x - cur.transform.translation.x
    err.transform.translation.y = ref.transform.translation.y - cur.transform.translation.y
    err.transform.translation.z = ref.transform.translation.z - cur.transform.translation.z

    new_euler = tuple(map(lambda i, j: i - j, euler_from_quaternion([ref.transform.rotation.x, ref.transform.rotation.y, ref.transform.rotation.z, ref.transform.rotation.w]), euler_from_quaternion([cur.transform.rotation.x, cur.transform.rotation.y, cur.transform.rotation.z, cur.transform.rotation.w])))

    [err.transform.rotation.x, err.transform.rotation.y, err.transform.rotation.z, err.transform.rotation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])

    '''
    err.transform.rotation.x = ref.transform.rotation.x - cur.transform.rotation.x
    err.transform.rotation.y = ref.transform.rotation.y - cur.transform.rotation.y
    err.transform.rotation.z = ref.transform.rotation.z - cur.transform.rotation.z
    err.transform.rotation.w = ref.transform.rotation.w - cur.transform.rotation.w
    '''

    return err

def correct_odom(odom_tf, err):
    new_tf = TransformStamped()
    new_tf.header = odom_tf.header
    new_tf.header.frame_id = "odom_correct"
    new_tf.child_frame_id = "odom"
    new_tf.transform = err.transform

    '''
    new_tf.transform.translation.x = odom_tf.transform.translation.x + err.transform.translation.x
    new_tf.transform.translation.y = odom_tf.transform.translation.y + err.transform.translation.y
    new_tf.transform.translation.z = odom_tf.transform.translation.z + err.transform.translation.z

    new_euler = tuple(map(lambda i, j: i + j, euler_from_quaternion([odom_tf.transform.rotation.x, odom_tf.transform.rotation.y, odom_tf.transform.rotation.z, odom_tf.transform.rotation.w]), euler_from_quaternion([err.transform.rotation.x, err.transform.rotation.y, err.transform.rotation.z, err.transform.rotation.w])))

    [new_tf.transform.rotation.x, new_tf.transform.rotation.y, new_tf.transform.rotation.z, new_tf.transform.rotation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])

    
    new_tf.transform.rotation.x = odom_tf.transform.rotation.x + err.transform.rotation.x
    new_tf.transform.rotation.y = odom_tf.transform.rotation.y + err.transform.rotation.y
    new_tf.transform.rotation.z = odom_tf.transform.rotation.z + err.transform.rotation.z
    new_tf.transform.rotation.w = odom_tf.transform.rotation.w + err.transform.rotation.w
    '''

    return new_tf

if __name__=="__main__":
    rospy.init_node("correct_error", anonymous=True)
    buf = tf.Buffer()
    ls = tf.TransformListener(buf)
    br = tf.TransformBroadcaster()
    new_odom_to_rob = None

    try:
        while not rospy.is_shutdown():
            try:
                cur_tf = buf.lookup_transform('odom', 'tag_world_cur', rospy.Time.now(), rospy.Duration(1.0))
                ref_tf = buf.lookup_transform('odom', 'tag_world_ref', rospy.Time.now(), rospy.Duration(1.0))
                err_tf = buf.lookup_transform('tag_world_ref', 'tag_world_cur', rospy.Time.now(), rospy.Duration(1.0))#find_error(cur_tf, ref_tf)
                odom_to_rob = buf.lookup_transform('odom', 'camera_link', rospy.Time.now(), rospy.Duration(1.0))
                new_odom_to_rob = correct_odom(odom_to_rob, err_tf)
                br.sendTransform(new_odom_to_rob)
                print(err_tf)
            except:
                if not new_odom_to_rob != None:
                    br.sendTransform(new_odom_to_rob)
                continue

    except KeyboardInterrupt:
        exit(0)

