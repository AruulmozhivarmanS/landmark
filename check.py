import rospy
import tf2_ros as tf
from utils import apply_transform
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

def publish_odom(trfm_msg, map_pub):
    pub_msg = Odometry()
    pub_msg.header = trfm_msg.header
    pub_msg.child_frame_id = trfm_msg.child_frame_id

    [pub_msg.pose.pose.position.x, pub_msg.pose.pose.position.y, pub_msg.pose.pose.position.z] = [trfm_msg.transform.translation.x, trfm_msg.transform.translation.y, trfm_msg.transform.translation.z]

    [pub_msg.pose.pose.orientation.x, pub_msg.pose.pose.orientation.y, pub_msg.pose.pose.orientation.z, pub_msg.pose.pose.orientation.w] = [trfm_msg.transform.rotation.x, trfm_msg.transform.rotation.y, trfm_msg.transform.rotation.z, trfm_msg.transform.rotation.w]

    map_pub.publish(pub_msg)

def publish_pose(trfm_msg, map_pub):
    pub_msg = PoseWithCovarianceStamped()
    pub_msg.header = trfm_msg.header

    [pub_msg.pose.pose.position.x, pub_msg.pose.pose.position.y, pub_msg.pose.pose.position.z] = [trfm_msg.transform.translation.x, trfm_msg.transform.translation.y, trfm_msg.transform.translation.z]

    [pub_msg.pose.pose.orientation.x, pub_msg.pose.pose.orientation.y, pub_msg.pose.pose.orientation.z, pub_msg.pose.pose.orientation.w] = [trfm_msg.transform.rotation.x, trfm_msg.transform.rotation.y, trfm_msg.transform.rotation.z, trfm_msg.transform.rotation.w]

    map_pub.publish(pub_msg)

def publish_map(m2r, t2m, pub):
    pub_msg = Odometry()
    pub_msg.header = t2m.header
    pub_msg.child_frame_id = m2r.child_frame_id
    
    trfm_msg = apply_transform(t2m, m2r)

    [pub_msg.pose.pose.position.x, pub_msg.pose.pose.position.y, pub_msg.pose.pose.position.z] = [trfm_msg.transform.translation.x, trfm_msg.transform.translation.y, trfm_msg.transform.translation.z]

    [pub_msg.pose.pose.orientation.x, pub_msg.pose.pose.orientation.y, pub_msg.pose.pose.orientation.z, pub_msg.pose.pose.orientation.w] = [trfm_msg.transform.rotation.x, trfm_msg.transform.rotation.y, trfm_msg.transform.rotation.z, trfm_msg.transform.rotation.w]

    pub.publish(pub_msg)

    
   
def publish_gt(gt_list, tag_pub):
    for trfm_msg in gt_list:
        pub_msg = Vector3()
        [pub_msg.x, pub_msg.y, pub_msg.z] = [trfm_msg.transform.translation.x, trfm_msg.transform.translation.y, trfm_msg.transform.translation.z]
        tag_pub.publish(pub_msg)

def odom_points(msg, args):
    buf = args[0]
    odom_pub = args[1]
    if len(msg.detections) > 1:
        odom_tf = buf.lookup_transform('tag_world_ref', 'base_link', rospy.Time(0), rospy.Duration(1))
        publish_odom(odom_tf, odom_pub)


if __name__=="__main__":
    rospy.init_node("correct_error", anonymous=True)
    map_pub = rospy.Publisher('tag_map/pose_pc', Odometry, queue_size=10)
    amcl_pub = rospy.Publisher('tag_map/amcl_pc', Odometry, queue_size=10)
    odom_pub = rospy.Publisher('tag_map/odom_filtered_pc', Odometry, queue_size=10)
    tag_pub = rospy.Publisher('tag_map/markers_pc', Vector3, queue_size=10)

    buf = tf.Buffer()
    ls = tf.TransformListener(buf)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)
    tag_list = ['0', '1', '2', '3', '4', '5', '16', '7', '8', '9', '10', '11', '17', '14', '15']
    location_list = []

    #rospy.Subscriber('/tag_detections', AprilTagDetectionArray, odom_points, (buf, odom_pub), queue_size = 100)

    while not rospy.is_shutdown():

        try:
            check_tf = buf.lookup_transform('tag_world_cur_pc', 'base_link', rospy.Time(0), rospy.Duration(1))
            amcl_tf = buf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1))
            map_tf = buf.lookup_transform('tag_world_ref_pc', 'map', rospy.Time(0), rospy.Duration(1))
            odom_tf = buf.lookup_transform('tag_world_ref_pc', 'base_link', rospy.Time(0), rospy.Duration(1))

            for tag in tag_list:
                location_list.append(buf.lookup_transform('tag_world_ref_pc', str(tag)+'_ref_pc', rospy.Time(0), rospy.Duration(1)))

            publish_odom(check_tf, map_pub) #Publish Tag_Pose
            publish_odom(odom_tf, odom_pub) #Publish Odometry in Tag Frame
            publish_map(amcl_tf, map_tf, amcl_pub) #Publish AMCL in Tag Frame
            publish_gt(location_list, tag_pub) #Publish Tag Locations

            rate.sleep()

        except:
            rate.sleep()
            continue
