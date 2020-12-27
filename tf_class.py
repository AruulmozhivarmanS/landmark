import copy
import rospy
import tf2_ros as tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from utils import apply_transform, convert_to_tf_static, read_gt, apply_transform_inv, sort_l2

class Landmarks_TF:
    def __init__(self, buf, ls, gt_file = None, suffix='', cur_frame='base_link'):

        if gt_file == None:
            print('Provide GT File!')
            exit(0)

        if cur_frame not in ['base_link', 'odom']:
            print('Only "base_link", "odom" frame-ids allowed!')
            exit(0)

        self.gts = read_gt(gt_file)
        self.buf = buf
        self.ls = ls
        self.frame_suffix = suffix
        self.cur_frame_id = cur_frame

        self.ref_tf = None # Reference tf used when tag is detected for the first time
        self.cur_tf = None # Subsequent tf used to compare drift from ref_tf
        self.prev_tag = None
        self.cam2id_tf = None
        self.id2org_tf = None
        self.can_update = False

    def get_gt(self, pose_msg, tag_name):
        static_tf_ref = convert_to_tf_static(pose_msg.pose, "tag_world_ref"+self.frame_suffix, tag_name+"_ref"+self.frame_suffix, pose_msg.header.seq)
        static_tf_cur = convert_to_tf_static(pose_msg.pose, "tag_world_cur"+self.frame_suffix, tag_name+"_cur"+self.frame_suffix, pose_msg.header.seq)

        return [static_tf_ref, static_tf_cur]

    def get_ref(self, tag_name):
        ref_cam2id_tf = None
        ref_id2org_tf = None

        while ref_cam2id_tf == None:
            try:
                ref_cam2id_tf = self.buf.lookup_transform("odom", "tag_"+tag_name, rospy.Time(0), rospy.Duration(0.1))
                ref_id2org_tf = self.get_gt(self.gts[tag_name], tag_name)
            except:
                continue
        print(ref_cam2id_tf, ref_id2org_tf)
        self.ref_tf = apply_transform_inv(ref_cam2id_tf, ref_id2org_tf[0])
        self.ref_tf.child_frame_id = "tag_world_ref"+self.frame_suffix
        self.cur_tf = copy.deepcopy(self.ref_tf)
        self.cur_tf.header.frame_id = self.cur_frame_id
        self.cur_tf.child_frame_id = "tag_world_cur"+self.frame_suffix


    def publish_static(self): # Publish according to sequence in the list (default)
        tfs = [self.ref_tf, self.cur_tf]
        br = tf.StaticTransformBroadcaster()
        for key, value in self.gts.items():
            tfs+=self.get_gt(value, key)
        br.sendTransform(tfs)

    def publish_dynamic(self):
        br = tf.TransformBroadcaster()
        try:
            br.sendTransform(self.cur_tf)
            self.can_update=True
            #print('Pose Updated')
        except:
            pass

    def cam_init(self, april_msg): # Publish according to sequence in /tag_detections topic from apriltag_ros         
        if len(april_msg.detections) > 0:
            if self.ref_tf == None:
                new_msg = sort_l2(april_msg.detections)
                first_tag_name = str(new_msg.id[0])
                if not first_tag_name == '20':
                    print("Tag ID "+first_tag_name+" seen.")
                    #self.prev_tag = first_tag_name
                    self.get_ref(first_tag_name) # Keep track of the first TF, make it Reference
                    self.publish_static() # Publish all TF together, as required by TF_Static
            else:
                self.update(april_msg)
    
    def update(self, new_april_msg):

        if len(new_april_msg.detections) > 1:
            detected_tag = sort_l2(new_april_msg.detections)
        else:
            detected_tag = new_april_msg.detections[0]

        for detect in [detected_tag]:
            tag_id = str(detect.id[0])

            if self.prev_tag == tag_id:
                continue

            else:
                self.cam2id_tf = self.buf.lookup_transform(self.cur_frame_id, "tag_"+tag_id, rospy.Time(0), rospy.Duration(0.1))
                self.id2org_tf = self.buf.lookup_transform(tag_id+"_cur"+self.frame_suffix, "tag_world_cur"+self.frame_suffix, rospy.Time(0), rospy.Duration(0.3))
                self.cam2id_tf.child_frame_id = tag_id+"_cur"+self.frame_suffix
                #self.prev_tag = tag_id
                print("------")
                print("Tag ID "+tag_id+" seen.")
                print("Got TF")

            if self.id2org_tf != None and self.cam2id_tf != None:
                self.cur_tf = apply_transform(self.cam2id_tf, self.id2org_tf)
                self.cam2id_tf = None
                self.id2org_tf = None
                self.can_update = True


