from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from utils import sort_l2
import message_filters
import tf2_ros as tf
import numpy as np
import rosbag
import rospy
import os
import pickle

save_dir = '/home/anushl9o5/Desktop/RBCCPS/Garrett/landmark_v2/left_side_data_man/13_npy'
saved = False

'''
for i in [1, 8, 12, 13]:
    if not os.path.isdir(os.path.join(save_dir, str(i)+'_npy')):
        os.makedirs(os.path.join(save_dir, str(i)+'_npy', 'data'))
        os.makedirs(os.path.join(save_dir, str(i)+'_npy', 'gt'))

exit()
'''

# origin_tag = '0'
# all_tags = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13',]

rospy.init_node("data_node", anonymous=True)

buf = tf.Buffer()
ls = tf.TransformListener(buf)

gt_global = []
data_global = []
tag_global = []

gt_np = []
data_np = []

def data_synced(tag, tag_odom):
    # global origin_tag

    global tag_global, data_global, gt_global
    gt_array = []
    tag_array = []
    data_array = []

    for tag in tag.detections:
        tag_id = str(tag)
        try:
            data_array.append(buf.lookup_transform('tag_'+tag_id, 'base_link', rospy.Time(0), rospy.Duration(0.01)))
            gt_array.append(tag_odom)
            tag_array.append(tag)
            print(tag_id)
        except:
            pass

    tag_global.append(tag_array)
    gt_global.append(gt_array)
    data_global.append(data_array)

def convert_npy():
    global gt_global, tag_global, data_global
    global gt_np, data_np

    for gts, odoms in zip(gt_global, data_global):
        gt_np.append(convert_msg_array(gts))
        data_global.append(convert_msg_array(odoms))
    
    with open("gt_1.txt", "wb") as fp:
        pickle.dump(gt_np, fp)
    
    with open("tag_1.txt", "wb") as fp:
        pickle.dump(tag_global, fp)
    
    with open("data_1.txt", "wb") as fp:
        pickle.dump(data_np, fp)
    


def convert_msg_array(msgs):
    msg_array = []
    for msg in msgs:
        msg_array.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    return msg_array

def save_data(msg):
    global saved
    if not saved:
        convert_npy()
        saved = True

if __name__ == "__main__":

    bag_done = rospy.Subscriber('/done', String, save_data)
    odom_sub = message_filters.Subscriber('/odometry/filtered', Odometry)
    tag_det = message_filters.Subscriber('/tag_detections', AprilTagDetectionArray)
    odom_in_tag = message_filters.Subscriber('/tag_map/odom_filtered', Odometry)

    topic_sync = message_filters.ApproximateTimeSynchronizer([tag_det, odom_in_tag], 10, 0.01)
    topic_sync.registerCallback(data_synced)

    rospy.spin()
