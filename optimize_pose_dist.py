from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, quaternion_inverse, quaternion_multiply
from utils import apply_transform, read_gt
from scipy.optimize import least_squares
from glob import glob
import numpy as np
import random
import os

data_dict = dict()
gt_dict = dict()
visited_dict = dict()

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
    transformed = dict()
    for run in data_dict:
        transformed[run] = dict()
        for key in data_dict[run]:
            if key != '0':
                tf_list = []
                for data_pt in data_dict[run][key]:
                    tf_list.append(forward_tf(data_pt, new_pose[key]))
                transformed[run][key] = np.asarray(tf_list)
            else:
                transformed[run][key] = data_dict[run][key]

    return transformed
        
def compute_dist_error(new, gt, num_samples=400):
    global visited_dict
    total_error = []
    error_dict = dict()

    for run in new:
        if not run in visited_dict.keys():
            visited_dict[run] = dict()
        for key in new[run]:
            num_samples = 400
            if key != '0':
                x_n1_t = new[run][key][:, :3]
                y_n1_t = gt[run][key][:, :3]
                
                if not key in visited_dict[run]:
                    key_n2 = random.sample(list(new[run]), 1)
                    while key == key_n2[0]:
                        key_n2 = random.sample(list(new[run]), 1)

                    visited_dict[run][key] = key_n2

                else:
                    key_n2 = visited_dict[run][key]

                y_n2_t = gt[run][key_n2[0]][:, :3]
                x_n2_t = new[run][key_n2[0]][:, :3]

                '''
                if len(x_n1_t) > len(x_n2_t):
                    if num_samples > len(x_n2_t):
                        num_samples = len(x_n2_t)
                else:
                    if num_samples > len(x_n1_t):
                        num_samples = len(x_n1_t)

                timesteps_1 = np.random.choice(len(x_n1_t), num_samples)
                timesteps_2 = np.random.choice(len(x_n2_t), num_samples)
                '''
                
                x_n1 = x_n1_t#[timesteps_1]
                x_n2 = x_n2_t#[timesteps_2]
                y_n1 = y_n1_t#[timesteps_1]
                y_n2 = y_n2_t#[timesteps_2]

                error = []
                for i in range(len(x_n2)):
                    for j in range(len(x_n1)):

                        x_l2_norm = (x_n1[j] - x_n2[i])**2
                        x_l2_norm = np.sqrt(x_l2_norm[0] + x_l2_norm[1])
                        
                        y_l2_norm = (y_n1[j] - y_n2[i])**2
                        y_l2_norm = np.sqrt(y_l2_norm[0] + y_l2_norm[1])

                        err = (y_l2_norm - x_l2_norm)**2
                        if err < 0.5:
                            error.append(err)

                if np.isnan(np.mean(np.asarray(error))):
                    print(key, key_n2, run, sorted(error)[:10])
                    visited_dict[run].pop(key)
                    continue

                try:
                    error_dict[key].append(np.mean(np.asarray(error)))
                except KeyError:
                    error_dict[key] = [np.mean(np.asarray(error))]

                #print(run, key, key_n2)

    for key in error_dict:
        print(key, np.mean(error_dict[key]))
        total_error.append(np.mean(error_dict[key]))
        total_error.append(np.mean(error_dict[key]))
        total_error.append(np.mean(error_dict[key]))
        total_error.append(np.mean(error_dict[key]))
        total_error.append(np.mean(error_dict[key]))

    return np.asarray(total_error)


if __name__ == "__main__":
    folders = sorted(glob('left_side_data/*_npy'))
    print(folders)
    gt = dict()
    data = dict()

    for i, folder in enumerate(folders):
        print(i, folder)
        gt[i] = glob(os.path.join(folder ,'gt/*.npy'))
        data[i] = glob(os.path.join(folder, 'data/*.npy'))

    assert len(gt) == len(data)

    #visited = {'1':False, '2':False, '3':False, '4':False, '5':False, '6':False}

    for i, run_name in enumerate(gt):
        data_dict[run_name] = dict()
        gt_dict[run_name] = dict()
        for j, fname in enumerate(gt[run_name]):
            tag_id = fname.split('/')[-1].split('.')[0]
            data_dict[run_name][tag_id] = np.load(data[run_name][j])
            gt_dict[run_name][tag_id] = np.load(fname)

    '''    
    for run, d in data_dict.items():
        print(run)
        for key, val in d.items():
            print(key, val.shape)

    exit(0)
    '''

    initial_tag_pose = read_gt('config/left_side_gmap_gt3.txt')
    theta0, tag_order, pose_dict = convert_initial(initial_tag_pose)

    def res(theta):

        new_pose = {}
        for i, tag_theta in enumerate(theta[::5]):
            quat = quaternion_from_euler(theta[(i*5)+2], theta[(i*5)+3], theta[(i*5)+4])
            new_pose[tag_order[i]] = pose_dict[tag_order[i]]
            new_pose[tag_order[i]][0], new_pose[tag_order[i]][1] = theta[i*5], theta[(i*5)+1]
            new_pose[tag_order[i]][3], new_pose[tag_order[i]][4], new_pose[tag_order[i]][5], new_pose[tag_order[i]][6] = quat[0], quat[1], quat[2], quat[3]

            print(tag_order[i], new_pose[tag_order[i]])

        write_file(new_pose)

        transformed_pose = compute_transform(new_pose)

        err = compute_dist_error(transformed_pose, gt_dict)

        return err 
    
    #res(theta0)
      
    for _ in range(1):
        result = least_squares(res, theta0, method='trf', ftol=1e-8)
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
        
