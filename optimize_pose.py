from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, quaternion_inverse, quaternion_multiply
from utils import apply_transform, read_gt
from scipy.optimize import least_squares
from glob import glob
import numpy as np
import random
import os

data_dict = dict()
gt_dict = dict()

def write_file(tags):
    f = open("config/optimized_pose/left_side_optim2.txt", 'w')
    f.write("id tx ty tz rz ry rz rw\n")
    for key, key_init in tags.items():
            f.write("%s %f %f %f %f %f %f %f\n"%(key, key_init[0]*100, key_init[1]*100, key_init[2]*100, key_init[3], key_init[4], key_init[5], key_init[6]))


def convert_initial(tag_pose):
    pose_dict = {}
    pose_list = []
    tag_order = []
    for key, value in tag_pose.items():
        if key != '0':
            pose_dict[key] = np.array([value.pose.position.x, value.pose.position.y, value.pose.position.z, value.pose.orientation.x, value.pose.orientation.y, value.pose.orientation.z, value.pose.orientation.w])
            euler = euler_from_quaternion([value.pose.orientation.x, value.pose.orientation.y, value.pose.orientation.z, value.pose.orientation.w])

            pose_list.append(pose_dict[key][0])
            pose_list.append(pose_dict[key][1])
            pose_list.append(euler[0])
            pose_list.append(euler[1])
            pose_list.append(euler[2])
            tag_order.append(key)

    return np.asarray(pose_list), tag_order, pose_dict

def forward_tf(from_tf, to_tf):
    def get_R(quat):
        R = quaternion_matrix(quat)
        return R[:3, :3]

    tf = [None]*7
    R = get_R([to_tf[3], to_tf[4], to_tf[5], to_tf[6]])
    [tf[0], tf[1], tf[2]] = [to_tf[0], to_tf[1], to_tf[2]] + np.matmul(R, [from_tf[0], from_tf[1], from_tf[2]])
    tf[3:] = quaternion_multiply(to_tf[3:], from_tf[3:])

    return np.asarray(tf)

def compute_transform(new_pose):
    global data_dict
    transformed = {}
    for key in data_dict:
        tf_list = []
        for data_pt in data_dict[key]:
            tf_list.append(forward_tf(data_pt, new_pose[key]))
        transformed[key] = np.asarray(tf_list)

    return transformed

def compute_pos_error(new, gt):
    total_error = []
    for key in new:
        err = (new[key][:, :3] - gt[key][:, :3])
        err = err[err[:, 0] < 0.1]
        err = err**2
        #if key == '3':
            #print(np.sort(err[:, 0])[-20:])

        total_error.append(np.mean(err))
        total_error.append(np.mean(err))
        total_error.append(np.mean(err))
        total_error.append(np.mean(err))
        total_error.append(np.mean(err))

        print(key, np.sqrt(total_error[-1])*100)

    return np.asarray(total_error)
        
if __name__ == "__main__":
    folders = sorted(glob('left_side_data_man/*_npy'))
    print(folders)
    gt = []
    data = []

    for folder in folders:
        gt += glob(os.path.join(folder ,'gt/*.npy'))
        data += glob(os.path.join(folder, 'data/*.npy'))

    assert len(gt) == len(data)

    visited = {'1':False, '2':False, '3':False, '4':False, '5':False, '16':False, '7':False, '8':False, '9':False, '10':False, '11':False, '17':False, '14':False, '15':False}

    for i, fname in enumerate(gt):
        tag_id = fname.split('/')[-1].split('.')[0]
        if tag_id != '0':
            if visited[tag_id]:
                data_dict[tag_id] = np.row_stack((data_dict[tag_id], np.load(data[i])))
                gt_dict[tag_id] = np.row_stack((gt_dict[tag_id], np.load(fname)))
            else:
                data_dict[tag_id] = np.load(data[i])
                gt_dict[tag_id] = np.load(fname)
                visited[tag_id] = True

    print(len(gt_dict['4']))

    initial_tag_pose = read_gt('config/left_side_gmap_gt4.txt')
    #initial_tag_pose = read_gt('config/optimized_pose/airl_flat_optim_curved.txt')
    theta0, tag_order, pose_dict = convert_initial(initial_tag_pose)

    def res(theta, test=False):
        #print('theta')
        #print(theta)

        new_pose = {}
        for i, tag_theta in enumerate(theta[::5]):
            quat = quaternion_from_euler(theta[(i*5)+2], theta[(i*5)+3], theta[(i*5)+4])
            new_pose[tag_order[i]] = pose_dict[tag_order[i]]
            new_pose[tag_order[i]][0], new_pose[tag_order[i]][1] = theta[i*5], theta[(i*5)+1]
            new_pose[tag_order[i]][3], new_pose[tag_order[i]][4], new_pose[tag_order[i]][5], new_pose[tag_order[i]][6] = quat[0], quat[1], quat[2], quat[3]

        write_file(new_pose)

        transformed_pose = compute_transform(new_pose)

        err = compute_pos_error(transformed_pose, gt_dict)

        if test:
            exit()

        return err 
    
    #res(theta0, True)
       
    for _ in range(1):
        result = least_squares(res, theta0, method='trf', ftol=1e-5)
        theta0 = result.x

    theta_final = result.x
    
    final_pose = {}
    for i, tag_theta in enumerate(theta_final[::5]):
        quat = quaternion_from_euler(theta_final[(i*5)+2], theta_final[(i*5)+3], theta_final[(i*5)+4])
        final_pose[tag_order[i]] = pose_dict[tag_order[i]]
        final_pose[tag_order[i]][0], final_pose[tag_order[i]][1] = theta_final[i*5], theta_final[(i*5)+1]
        final_pose[tag_order[i]][3], final_pose[tag_order[i]][4], final_pose[tag_order[i]][5], final_pose[tag_order[i]][6] = quat[0], quat[1], quat[2], quat[3]

    
    for key, value in final_pose.items():
        print(key, value)
        
