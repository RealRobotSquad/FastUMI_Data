import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import cv2
from tqdm import tqdm
from multiprocessing import Pool, cpu_count
import json

# Load the configuration from the config.json file
with open('config/config.json', 'r') as config_file:
    config = json.load(config_file)
config = config["data_process_config"]

# Load predefined ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["aruco_dict"]))
parameters = cv2.aruco.DetectorParameters()

def get_gripper_width(img_list):
    """
    Calculate gripper width from detected ArUco markers in the images.
    """
    distances = []
    distances_index = []
    current_frame = 0
    frame_count = len(img_list)
    current_frame += 1
    for i in range(img_list.shape[0]):
        gray = cv2.cvtColor(img_list[i, :, :, :], cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            
            marker_centers = []
            for idx, marker_id in enumerate(ids.flatten()):
                if marker_id in [config["marker_id_0"], config["marker_id_1"]]:
                    marker_corners = corners[idx][0]
                    center = np.mean(marker_corners, axis=0).astype(int)
                    marker_centers.append(center)
            
            if len(marker_centers) >= 2:
                distance = np.linalg.norm(marker_centers[0] - marker_centers[1])
                distances.append(distance)
                distances_index.append(current_frame)
            elif len(marker_centers) == 1:
                distance = abs(gray.shape[1] / 2 - marker_centers[0][0]) * 2
                distances.append(distance)
                distances_index.append(current_frame)

    distances = np.array(distances)
    distances_index = np.array(distances_index)
    distances = ((distances - config["distances"]["marker_min"]) / (config["distances"]["marker_max"] - config["distances"]["marker_min"]) * config["distances"]["gripper_max"]).astype(np.int16).clip(0, config["distances"]["gripper_max"])
    
    new_distances = []
    for i in range(len(distances) - 1):
        if i == 0:
            if distances_index[i] == 1:
                new_distances.append(distances[0])
                continue
            else:
                for _ in range(distances_index[0]):
                    new_distances.append(distances[0])
        else:
            if distances_index[i + 1] - distances_index[i]==1:
                new_distances.append(distances[i])
            else:
                for k in range(distances_index[i + 1] - distances_index[i]):
                    interpolated_distance = int(
                        k * (distances[i + 1] - distances[i]) /
                        (distances_index[i + 1] - distances_index[i]) +
                        distances[i])
                    new_distances.append(interpolated_distance)
    new_distances.append(distances[-1])
    if len(new_distances) < frame_count:
        for _ in range(frame_count - len(new_distances)):
            new_distances.append(distances[-1])
    
    return np.array(new_distances)

def transform_to_base_quat(x, y, z, qx, qy, qz, qw, T_base_to_local):
    rotation_local = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T_local = np.eye(4)
    T_local[:3, :3] = rotation_local
    T_local[:3, 3] = [x, y, z]
    T_base_r = np.dot(T_local[:3, :3] , T_base_to_local[:3, :3] )
    x_base, y_base, z_base = T_base_to_local[:3, 3] + T_local[:3, 3]
    rotation_base = R.from_matrix(T_base_r)
    roll_base, pitch_base, yaw_base = rotation_base.as_euler('xyz', degrees=False)
    qx_base, qy_base, qz_base, qw_base = rotation_base.as_quat()
    
    return x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base, roll_base, pitch_base, yaw_base


def normalize_and_save_base_tcp_hdf5(args):
    input_file, output_file = args
    base_x, base_y, base_z = config["base_position"]["x"], config["base_position"]["y"], config["base_position"]["z"] # Initial position of the robot's base in 3D space (in meters)
    base_roll, base_pitch, base_yaw = np.deg2rad([config["base_orientation"]["roll"], config["base_orientation"]["pitch"], config["base_orientation"]["yaw"]]) # Initial orientation of the robot's base in 3D space (in roll, pitch, yaw format) (in degrees)
    rotation_base_to_local = R.from_euler('xyz', [base_roll, base_pitch, base_yaw]).as_matrix()
    
    T_base_to_local = np.eye(4)
    T_base_to_local[:3, :3] = rotation_base_to_local
    T_base_to_local[:3, 3] = [base_x, base_y, base_z]
    
    try:
        with h5py.File(input_file, 'r') as f_in:
            action_data = f_in['action'][:]
            qpos_data = f_in['observations/qpos'][:]
            image_data = f_in['observations/images/front'][:]             
            normalized_qpos = np.copy(qpos_data)

            for i in range(normalized_qpos.shape[0]):
                x, y, z, qx, qy, qz, qw = normalized_qpos[i, 0:7]
                x -= config["offset"]["x"]
                z += config["offset"]["z"]

                x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base, _, _, _ = transform_to_base_quat(x, y, z, qx, qy, qz, qw, T_base_to_local)
                ori = R.from_quat([qx_base, qy_base, qz_base, qw_base]).as_matrix()
                pos = np.array([x_base, y_base, z_base])
                pos += config["offset"]["x"] * ori[:, 2] 
                pos -= config["offset"]["z"] * ori[:, 0]
                x_base, y_base, z_base = pos
                normalized_qpos[i, :] = [x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base]

            image_data = np.array(image_data)
            gripper_open_width = get_gripper_width(image_data)
            gripper_open_width = gripper_open_width / config["distances"]["gripper_max"]

            gripper_width = gripper_open_width.reshape(-1, 1)
            normalized_qpos_with_gripper = np.concatenate((normalized_qpos, gripper_width), axis=1)
            normalized_action_with_gripper = np.copy(normalized_qpos_with_gripper)

            with h5py.File(output_file, 'w') as f_out:
                f_out.create_dataset('action', data=normalized_action_with_gripper)
                observations_group = f_out.create_group('observations')
                images_group = observations_group.create_group('images')
                
                max_timesteps = f_in['observations/images/front'].shape[0]
                cam_hight, cam_width = f_in['observations/images/front'].shape[1], f_in['observations/images/front'].shape[2]

                images_group.create_dataset(
                    'front',
                    (max_timesteps, cam_hight, cam_width, 3),
                    dtype='uint8',
                    chunks=(1, cam_hight, cam_width, 3),
                    compression='gzip',
                    compression_opts=4
                )
                images_group['front'][:] = f_in['observations/images/front'][:]
                observations_group.create_dataset('qpos', data=normalized_qpos_with_gripper)
                                
                print(f"Normalized data saved to: {output_file}")
    except Exception as e:
        print(f"Error processing {input_file}: {e}")

if __name__ == "__main__":
    input_dir = config["input_dir"]
    output_dir = config["output_tcp_dir"]

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    file_list = [
        filename for filename in os.listdir(input_dir)
        if filename.endswith('.hdf5')
    ] 
    args_list = []
    for filename in file_list:
        input_file = os.path.join(input_dir, filename)
        output_file = os.path.join(output_dir, filename)
        args_list.append((input_file, output_file))

    print("Starting parallel processing...")

    num_processes = cpu_count()
    with Pool(num_processes) as pool:
        list(
            tqdm(pool.imap_unordered(normalize_and_save_base_tcp_hdf5, args_list),
                    total=len(args_list),
                    desc="Processing files"))

    print("Processing completed.")