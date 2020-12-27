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

save_dir = '/home/anushl9o5/Desktop/RBCCPS/Garrett/landmark_v2/left_side_data_man/13_npy'
saved = False

'''
for i in [1, 8, 12, 13]:
    if not os.path.isdir(os.path.join(save_dir, str(i)+'_npy')):
        os.makedirs(os.path.join(save_dir, str(i)+'_npy', 'data'))
        os.makedirs(os.path.join(save_dir, str(i)+'_npy', 'gt'))

exit()
'''

origin_tag = '0'
all_tags = ['0', '1', '2', '3', '4', '5', '16', '7', '8', '9', '10', '11', '17', '14', '15']

rospy.init_node("data_node", anonymous=True)

buf = tf.Buffer()
ls = tf.TransformListener(buf)

gt_dict = dict()
data_dict = dict()


def data_synced(tag, tag_odom):
    global origin_tag, data_dict, gt_dict

    if len(tag.detections) > 0:
        if len(tag.detections) > 1:
            detected_tag = sort_l2(tag.detections)
        else:
            detected_tag = tag.detections[0]

        tag_id = str(detected_tag.id[0])
        try:
            data_dict[tag_id].append(buf.lookup_transform('tag_'+tag_id, 'base_link', rospy.Time(0), rospy.Duration(0.01)))
            gt_dict[tag_id].append(tag_odom)
            print(tag_id)

        except:
            pass

def convert_npy():
    global gt_dict, data_dict, save_dir

    for key in gt_dict:
        gt_list = []
        for gt_msgs in gt_dict[key]:
            gt_list.append([gt_msgs.pose.pose.position.x, gt_msgs.pose.pose.position.y, gt_msgs.pose.pose.position.z, gt_msgs.pose.pose.orientation.x, gt_msgs.pose.pose.orientation.y, gt_msgs.pose.pose.orientation.z, gt_msgs.pose.pose.orientation.w])
        
        gt_list = np.asarray(gt_list)
        if len(gt_list) > 0:
            print(gt_list.shape)
            np.save(os.path.join(save_dir, 'gt', key+'.npy'), gt_list)

    for key in data_dict:
        data_list = []
        for data_msgs in data_dict[key]:
            data_list.append([data_msgs.transform.translation.x, data_msgs.transform.translation.y, data_msgs.transform.translation.z, data_msgs.transform.rotation.x, data_msgs.transform.rotation.y, data_msgs.transform.rotation.z, data_msgs.transform.rotation.w])
        
        data_list = np.asarray(data_list)
        if len(data_list) > 0:
            print(data_list.shape)
            np.save(os.path.join(save_dir, 'data', key+'.npy'), data_list)


def save_data(msg):
    global gt_dict, data_dict, saved
    if not saved:
        for key in gt_dict:
            print(len(gt_dict[key]), len(data_dict[key]))
        convert_npy()
        saved = True



if __name__ == "__main__":

    for tag_id in all_tags:
        #if tag_id == origin_tag:
        data_dict[tag_id] = []
        gt_dict[tag_id] = []

    bag_done = rospy.Subscriber('/done', String, save_data)
    odom_sub = message_filters.Subscriber('/odometry/filtered', Odometry)
    tag_det = message_filters.Subscriber('/tag_detections', AprilTagDetectionArray)
    odom_in_tag = message_filters.Subscriber('/tag_map/odom_filtered', Odometry)

    topic_sync = message_filters.ApproximateTimeSynchronizer([tag_det, odom_in_tag], 10, 0.01)
    topic_sync.registerCallback(data_synced)

    rospy.spin()
