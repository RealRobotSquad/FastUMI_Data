import h5py
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import ikpy.chain
import cv2
import ikpy.utils.plot as plot_utils
from tqdm import tqdm
from multiprocessing import Pool, cpu_count
import json

# Load the configuration from the config.json file
with open('config/config.json', 'r') as config_file:
    config = json.load(config_file)
config = config["data_process_config"]

# Extract configuration values
START_QPOS = config["start_qpos"] # Initial joint positions for the robot (values specific to your robot's configuration)
PI = np.pi

# Load the robot chain
my_chain = ikpy.chain.Chain.from_urdf_file(config["urdf_path"], base_elements=['world']) # Path to the URDF file of the robot model (replace with your robot's URDF file in config.json)

# Load predefined ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["aruco_dict"]))
parameters = cv2.aruco.DetectorParameters()

print(f"Number of joints in the chain: {len(my_chain)}")


def calculate_new_pose(x, y, z, quaternion, distance):
    """
    Calculate a new pose by translating along the negative Z-axis of the given pose.
    """
    rotation = R.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()
    z_axis = rotation_matrix[:, 2]
    new_position = np.array([x, y, z]) - distance * z_axis
    return [new_position[0], new_position[1], new_position[2]], quaternion


def cartesian_to_joints(position, quaternion, initial_joint_angles=None, **kwargs):
    """
    Convert Cartesian coordinates to robot joint angles using inverse kinematics.
    """
    rotation = R.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()

    if initial_joint_angles is None:
        initial_joint_angles = [0] * len(my_chain)

    joint_angles = my_chain.inverse_kinematics(
        position,
        rotation_matrix,
        orientation_mode='all',
        initial_position=initial_joint_angles
    )

    return joint_angles


def get_gripper_width(img_list):
    """
    Calculate gripper width from detected ArUco markers in the images.
    """
    distances = []
    distances_index = []
    current_frame = 0
    frame_count = len(img_list)

    for i in range(img_list.shape[0]):
        gray = cv2.cvtColor(img_list[i, :, :, :], cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        current_frame += 1
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
            if distances_index[i + 1] - distances_index[i] == 1:
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
    T_base_r = np.dot(T_local[:3, :3], T_base_to_local[:3, :3])
    x_base, y_base, z_base = T_base_to_local[:3, 3] + T_local[:3, 3]
    rotation_base = R.from_matrix(T_base_r)
    roll_base, pitch_base, yaw_base = rotation_base.as_euler(
        'xyz', degrees=False)
    qx_base, qy_base, qz_base, qw_base = rotation_base.as_quat()
    return x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base, roll_base, pitch_base, yaw_base


def normalize_ik_and_save_hdf5(args):
    """
    Normalize input data and save processed HDF5 files.
    """
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
            normalized_qpos = np.copy(qpos_data)

            for i in range(normalized_qpos.shape[0]):
                x, y, z, qx, qy, qz, qw = normalized_qpos[i, 0:7]
                x -= config["offset"]["x"]
                z += config["offset"]["z"]
                x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base, _, _, _ = transform_to_base_quat(
                    x, y, z, qx, qy, qz, qw, T_base_to_local)
                ori = R.from_quat([qx_base, qy_base, qz_base, qw_base]).as_matrix()
                pos = np.array([x_base, y_base, z_base])
                pos += config["offset"]["x"] * ori[:, 2]
                pos -= config["offset"]["z"] * ori[:, 0]
                x_base, y_base, z_base = pos
                normalized_qpos[i, :] = [x_base, y_base, z_base, qx_base, qy_base, qz_base, qw_base]

            joint_angles = []
            action_data = np.copy(normalized_qpos)
            image_data = f_in['observations/images/front'][:]
            qpos_data = normalized_qpos
            data = np.array(action_data)

            initial_joint_angles = None
            for i in range(len(data)):
                pose = data[i]
                direction = np.array(pose[:3])
                q = np.array(pose[3:])

                direction, quaternion = calculate_new_pose(
                    direction[0], direction[1], direction[2], q, config["distances"]["flange_to_tcp"])
                if i == 0:
                    initial_joint_angles = np.array(START_QPOS)
                    full_joint_angles = cartesian_to_joints(
                        direction, quaternion, initial_joint_angles)
                else:
                    full_joint_angles = cartesian_to_joints(
                        direction, quaternion, initial_joint_angles)
                initial_joint_angles = full_joint_angles
                six_dof_joint_angles = full_joint_angles[2:]
                joint_angles.append(six_dof_joint_angles)

            joint_angles = np.array(joint_angles)

            image_data = np.array(image_data)
            gripper_open_width = get_gripper_width(image_data)
            gripper_open_width = gripper_open_width / config["distances"]["gripper_max"]

            gripper_width = gripper_open_width.reshape(-1, 1)
            new_joint_angles = np.concatenate(
                (joint_angles, gripper_width), axis=1)

            with h5py.File(output_file, 'w') as f_out:
                f_out.create_dataset('action', data=new_joint_angles)
                observations_group = f_out.create_group('observations')
                images_group = observations_group.create_group('images')

                max_timesteps = f_in['observations/images/front'].shape[0]
                cam_hight = f_in['observations/images/front'].shape[1]
                cam_width = f_in['observations/images/front'].shape[2]

                images_group.create_dataset(
                    'front',
                    (max_timesteps, cam_hight, cam_width, 3),
                    dtype='uint8',
                    chunks=(1, cam_hight, cam_width, 3),
                    compression='gzip',
                    compression_opts=4)
                images_group['front'][:] = f_in['observations/images/front'][:]

                observations_group.create_dataset('qpos', data=new_joint_angles)

                print(f"Processed and saved: {output_file}")
    except Exception as e:
        print(f"Error processing {input_file}: {e}")


if __name__ == "__main__":
    input_dir = config["input_dir"]
    output_dir = config["output_joint_dir"]

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    file_list = [f for f in os.listdir(input_dir) if f.endswith('.hdf5')]
    args_list = []
    for f in file_list:
        input_file = os.path.join(input_dir, f)
        output_file = os.path.join(output_dir, f)
        args_list.append((input_file, output_file))
    print("Starting parallel processing...")

    num_processes = cpu_count()
    with Pool(num_processes) as pool:
        list(tqdm(pool.imap_unordered(normalize_ik_and_save_hdf5, args_list), total=len(args_list), desc="Processing files"))

    print("Processing completed.")
