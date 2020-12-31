import time
import rospy
import tf2_ros as tf
from utils import apply_transform, convert_to_tf_static, read_gt, apply_transform_gt
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped

class Landmarks_TF:
    def __init__(self, buf, ls):
        self.origin_id = None
        self.tagnames = ['5', '4', '6', '8', '7', '1', '3'] #['3', '5', '4']#
        self.ref_tf = None # Reference tf used when tag is detected for the first time
        self.cur_tf = None # Subsequent tf used to compare drift from ref_tf
        self.cam2id_tf = None
        self.id2org_tf = None
        self.gts = read_gt('config/new_airl_gt.txt')
        self.can_update = False
        self.buf = buf
        self.ls = ls
        self.prev_tag = None

    def get_gt(self, pose_msg, tag_name):
        if tag_name == self.origin_id:
            origin_pose = Pose()
            origin_pose.orientation.w = 1.0
            static_tf_ref = convert_to_tf_static(origin_pose, "tag_world_ref", tag_name+"_ref")
            static_tf_cur = convert_to_tf_static(origin_pose, "tag_world_cur", tag_name+"_cur")

        else:
            if self.origin_id == self.tagnames[0]: #if GT frame of reference is identical to first detection
                static_tf_ref = convert_to_tf_static(pose_msg.pose, "tag_world_ref", tag_name+"_ref", pose_msg.header.seq)
                static_tf_cur = convert_to_tf_static(pose_msg.pose, "tag_world_cur", tag_name+"_cur", pose_msg.header.seq)

            else: #if GT frame of reference is not identical to first detection
                new_pose = apply_transform_gt(pose_msg, self.gts[self.origin_id])
                static_tf_ref = convert_to_tf_static(new_pose.pose, "tag_world_ref", tag_name+"_ref", new_pose.header.seq)
                static_tf_cur = convert_to_tf_static(new_pose.pose, "tag_world_cur", tag_name+"_cur", new_pose.header.seq)

        return [static_tf_ref, static_tf_cur]

    def get_ref(self):
        ref_cam2id_tf = None
        while ref_cam2id_tf == None:
            try:
                ref_cam2id_tf = self.buf.lookup_transform("odom", "tag_"+self.origin_id, rospy.Time(0), rospy.Duration(0.01))
            except:
                continue
        print("Got Ref TF")
        self.ref_tf = ref_cam2id_tf
        self.ref_tf.child_frame_id = "tag_world_ref"
        self.cur_tf = ref_cam2id_tf
        #self.cur_tf.child_frame_id = "tag_world_cur"


    def publish_static(self): #Publish according to sequence in the list (default)
        tfs = [self.ref_tf]
        br = tf.StaticTransformBroadcaster()
        for key, value in self.gts.items():
            tfs+=self.get_gt(value, key)
        br.sendTransform(tfs)

    def publish_dynamic(self):
        br = tf.TransformBroadcaster()
        try:
            br.sendTransform(self.cur_tf)
            self.can_update = False
            print("Updated")
        except:
            pass

    def cam_init(self, april_msg): #Publish according to sequence in /tag_detections topic from apriltag_ros         
        #print(april_msg)
        if len(april_msg.detections) > 0:
            if self.origin_id == None and self.ref_tf == None:
                origin_tag_name = str(april_msg.detections[0].id[0])
                print("Origin Tag", origin_tag_name)
                self.origin_id = origin_tag_name
                self.prev_tag = origin_tag_name
                self.get_ref() #Keep track of the first TF, make it Reference
                self.publish_static() #Publish all TF together, as required by TF_Static
            else:
                self.update(april_msg)
    
    def update(self, new_april_msg):
        for detect in new_april_msg.detections:
            tag_id = str(detect.id[0])

            if self.prev_tag == tag_id:
                continue
            else:
                self.cam2id_tf = self.buf.lookup_transform("odom", "tag_"+tag_id, rospy.Time(0), rospy.Duration(0.01))
                self.id2org_tf = self.buf.lookup_transform(tag_id+"_cur", "tag_world_cur", rospy.Time(0), rospy.Duration(0.01))
                self.cam2id_tf.child_frame_id = tag_id+"_cur"
                self.prev_tag = tag_id
                print("Got TF")

            if self.id2org_tf != None and self.cam2id_tf != None:
                self.cur_tf = apply_transform(self.cam2id_tf, self.id2org_tf)
                self.can_update = True

