import rospy
import time
import tf2_ros as tf
from tf_class import Landmarks_TF
from apriltag_ros.msg import AprilTagDetectionArray


if __name__=="__main__":
    rospy.init_node("tag_tf_broadcaster", anonymous=True)
    rate = rospy.Rate(100)
    buf = tf.Buffer()
    ls = tf.TransformListener(buf)

    #gt = 'config/optimized_pose/left_side_optim2.txt'
    gt = 'config/left_side_gmap_gt.txt'

    ldmk = Landmarks_TF(buf, ls, gt_file = gt, suffix = '_pc', cur_frame = 'odom')
    tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ldmk.cam_init)

    while not rospy.is_shutdown():
        if ldmk.can_update:
            ldmk.publish_dynamic() #Publish dynamic TF
        rate.sleep()

    
