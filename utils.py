import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


def convert_to_tf_static(pose_msg, parent, child, header=0):
    tf_stamped = TransformStamped()
    tf_stamped.header.seq = header
    now = rospy.Time.now()
    tf_stamped.header.stamp.secs, tf_stamped.header.stamp.nsecs = now.secs, now.nsecs
    tf_stamped.header.frame_id = parent
    tf_stamped.child_frame_id = child
    tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf_stamped.transform.translation.z = pose_msg.position.x, pose_msg.position.y, pose_msg.position.z
    tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w = pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w

    return tf_stamped


def apply_transform(from_tf, to_tf):

    def get_R(trfm):
        quat_list = trfm.transform.rotation.x, trfm.transform.rotation.y, trfm.transform.rotation.z, trfm.transform.rotation.w
        rot_matrix = quaternion_matrix(quat_list)
        return rot_matrix[:3, :3]

    new_tf = TransformStamped()
    new_tf.header = from_tf.header
    new_tf.child_frame_id = to_tf.child_frame_id

    R = get_R(from_tf)

    #print(np.matmul(R,  [to_tf.transform.translation.x, to_tf.transform.translation.y, to_tf.transform.translation.z]))

    [new_tf.transform.translation.x, new_tf.transform.translation.y, new_tf.transform.translation.z] = [from_tf.transform.translation.x, from_tf.transform.translation.y, from_tf.transform.translation.z] - np.matmul(R,  [to_tf.transform.translation.x, to_tf.transform.translation.y, to_tf.transform.translation.z])

    new_euler = tuple(map(lambda i, j: i - j, euler_from_quaternion([from_tf.transform.rotation.x, from_tf.transform.rotation.y, from_tf.transform.rotation.z, from_tf.transform.rotation.w]), euler_from_quaternion([to_tf.transform.rotation.x, to_tf.transform.rotation.y, to_tf.transform.rotation.z, to_tf.transform.rotation.w])))

    [new_tf.transform.rotation.x, new_tf.transform.rotation.y, new_tf.transform.rotation.z, new_tf.transform.rotation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])

    return new_tf            

def apply_transform_new(from_tf, to_tf):

    def get_R(trfm):
        quat_list = trfm.transform.rotation.x, trfm.transform.rotation.y, trfm.transform.rotation.z, trfm.transform.rotation.w
        rot_matrix = quaternion_matrix(quat_list)
        return rot_matrix[:3, :3]

    new_tf = TransformStamped()
    new_tf.header = from_tf.header
    new_tf.child_frame_id = to_tf.child_frame_id

    R = get_R(from_tf)

    [new_tf.transform.translation.x, new_tf.transform.translation.y, new_tf.transform.translation.z] = np.matmul(R.transpose(),  [to_tf.transform.translation.x, to_tf.transform.translation.y, to_tf.transform.translation.z]) - [from_tf.transform.translation.x, from_tf.transform.translation.y, from_tf.transform.translation.z]

    new_euler = tuple(map(lambda i, j: i - j, euler_from_quaternion([from_tf.transform.rotation.x, from_tf.transform.rotation.y, from_tf.transform.rotation.z, from_tf.transform.rotation.w]), euler_from_quaternion([to_tf.transform.rotation.x, to_tf.transform.rotation.y, to_tf.transform.rotation.z, to_tf.transform.rotation.w])))

    [new_tf.transform.rotation.x, new_tf.transform.rotation.y, new_tf.transform.rotation.z, new_tf.transform.rotation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])

    return new_tf            


def apply_transform_gt(pose_msg, origin_msg):

    def get_T(org):
        quat_list = [org.pose.orientation.x, org.pose.orientation.y, org.pose.orientation.z, org.pose.orientation.w]
        rot_matrix = quaternion_matrix(quat_list)
        return rot_matrix[:3, :3]


    R = get_R(origin_msg)
    new_pose = PoseStamped()
    new_pose.header = pose_msg.header
    [new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z] = np.matmul(R, [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]) - np.matmul(R, [origin_msg.pose.position.x, origin_msg.pose.position.y, origin_msg.pose.position.z])

    new_euler =tuple(map(lambda i, j: i - j, euler_from_quaternion([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]), euler_from_quaternion([origin_msg.pose.orientation.x, origin_msg.pose.orientation.y, origin_msg.pose.orientation.z, origin_msg.pose.orientation.w])))
    
    [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])
    
    return new_pose

def apply_transform_gt_new(pose_msg, origin_msg):

    def get_T(org):
        quat_list = [org.pose.orientation.x, org.pose.orientation.y, org.pose.orientation.z, org.pose.orientation.w]
        rot_matrix = quaternion_matrix(quat_list)
        return rot_matrix[:3, :3]


    R = get_R(origin_msg)
    new_pose = PoseStamped()
    new_pose.header = pose_msg.header
    [new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z] = np.matmul(R.transpose(), [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]) - np.matmul(R.transpose(), [origin_msg.pose.position.x, origin_msg.pose.position.y, origin_msg.pose.position.z])

    new_euler =tuple(map(lambda i, j: i - j, euler_from_quaternion([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]), euler_from_quaternion([origin_msg.pose.orientation.x, origin_msg.pose.orientation.y, origin_msg.pose.orientation.z, origin_msg.pose.orientation.w])))
    
    [new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w] = quaternion_from_euler(new_euler[0], new_euler[1], new_euler[2])
    
    return new_pose


def read_gt(file_path):
    f = open(file_path, 'r')
    lines = f.readlines()
    gt_dict = {}
    for i, line in enumerate(lines[1:]):
        entries = line.rstrip().split(' ')
        tag_pose = PoseStamped()
        tag_pose.header.seq = i
        tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z = float(entries[1])/100.0, float(entries[3])/100.0, float(entries[2])/100.0
        tag_pose.pose.orientation.x, tag_pose.pose.orientation.y, tag_pose.pose.orientation.z, tag_pose.pose.orientation.w = float(entries[4]), float(entries[5]), float(entries[6]), float(entries[7])

        gt_dict[entries[0]] = tag_pose

    return gt_dict


if __name__ == "__main__":
   read_gt('config/manual_GT.txt') 
