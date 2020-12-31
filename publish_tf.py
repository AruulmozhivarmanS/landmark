import rospy
import time
import tf2_ros as tf
from apriltag_ros.msg import AprilTagDetectionArray
from tf_class import Landmarks_TF


if __name__=="__main__":
    rospy.init_node("tag_tf_broadcaster", anonymous=True)
    rate = rospy.Rate(0.1)
    buf = tf.Buffer()
    ls = tf.TransformListener(buf)

    ldmk = Landmarks_TF(buf, ls)
    tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, ldmk.cam_init)

    try:
        while not rospy.is_shutdown():
            if ldmk.can_update:
                ldmk.publish_dynamic() #Publish dynamic TF
        rate.sleep()

    except KeyboardInterrupt:
        exit(0)
    
