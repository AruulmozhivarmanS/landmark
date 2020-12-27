import rospy
import time
import tf2_ros as tf
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

tags = {}
buf = None
ls = None

def get_tag_loc(msg):
    global buf, tags
    if len(msg.detections) > 0:
        for detection in msg.detections:
            tag_id = str(detection.id[0])
            print("Tag "+tag_id)
            tfrm = buf.lookup_transform("map", "tag_"+tag_id, rospy.Time(0), rospy.Duration(0.1))
            euler = euler_from_quaternion([tfrm.transform.rotation.x, tfrm.transform.rotation.y, tfrm.transform.rotation.z, tfrm.transform.rotation.w])
            init_loc = np.array([tfrm.transform.translation.x, tfrm.transform.translation.y, tfrm.transform.translation.z, euler[0], euler[1], euler[2]])
            try:
                tags[tag_id].append(init_loc)
            except KeyError:
                tags[tag_id] = [init_loc]

def average_init_pose():
    global tags
    f = open("config/left_side_gmap_gt3.txt", 'w')
    f.write("id tx ty tz rz ry rz rw\n")
    for key, val in tags.items():
        if len(val) > 0:
            key_init = np.asarray(val).mean(axis=0)
            quat = quaternion_from_euler(key_init[3], key_init[4], key_init[5])
            f.write("%s %f %f %f %f %f %f %f\n"%(key, key_init[0]*100, key_init[1]*100, key_init[2]*100, quat[0], quat[1], quat[2], quat[3]))

if __name__=="__main__":
    rospy.init_node("tag_loc_initializer", anonymous=True)
    rate = rospy.Rate(25)
    buf = tf.Buffer()
    ls = tf.TransformListener(buf)

    tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, get_tag_loc)

    try:
        while not rospy.is_shutdown():
            rate.sleep()

        average_init_pose()

    except KeyboardInterrupt:
        pass

