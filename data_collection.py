import os
import json
import torch
import cv2
import h5py
import argparse
from tqdm import tqdm
from time import sleep
import numpy as np
import pyrealsense2 as rs
import apriltag
import rospy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import csv
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque
from datetime import datetime
import pandas as pd

# Load configuration from config.json
with open('config/config.json', 'r') as f:
    config = json.load(f)

# Set environment variables
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = "1"

# Set device
if torch.cuda.is_available():
    device = 'cuda'
else:
    device = 'cpu'

ROBOT_TYPE = config['device_settings']["robot_type"]
TASK_CONFIG = config['task_config']


# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--task', type=str, default="test3")  # open_lid, open_fridge, open_drawer, pick_place_pot
parser.add_argument('--num_episodes', type=int, default=2)
args = parser.parse_args()
task = args.task
num_episodes = args.num_episodes

cfg = TASK_CONFIG
robot = ROBOT_TYPE

data_path = os.path.join(config['device_settings']["data_dir"], "dataset" ,str(task))
os.makedirs(data_path, exist_ok=True)

IMAGE_PATH = os.path.join(data_path, 'camera/')
os.makedirs(IMAGE_PATH, exist_ok=True)

CSV_PATH = os.path.join(data_path, 'csv/')
os.makedirs(CSV_PATH, exist_ok=True)

STATE_PATH = os.path.join(data_path, 'states.csv')
if not os.path.exists(STATE_PATH):
    with open(STATE_PATH, 'w') as csv_file2:
        csv_writer2 = csv.writer(csv_file2)
        csv_writer2.writerow(['Index', 'Start Time', 'Trajectory Timestamp', 'Frame Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Q_X', 'Q_Y', 'Q_Z', 'Q_W'])

VIDEO_PATH_TEMP = os.path.join(data_path, 'camera', 'temp_video_n.mp4')
TRAJECTORY_PATH_TEMP = os.path.join(data_path, 'csv', 'temp_trajectory.csv')
TIMESTAMP_PATH_TEMP = os.path.join(data_path, 'csv', 'temp_video_timestamps.csv')
FRAME_TIMESTAMP_PATH_TEMP = os.path.join(data_path, 'csv', 'frame_timestamps.csv')

video_subscriber = None
trajectory_subscriber = None

# Initialize ROS node
rospy.init_node('video_trajectory_recorder', anonymous=True)

# Video writer parameters for 60 Hz recording
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
frame_width, frame_height = cfg['cam_width'], cfg['cam_height']

# Buffers for storing incoming data
video_buffer = deque()
trajectory_buffer = deque()

# Lock for thread synchronization
buffer_lock = threading.Lock()

# Initialize CvBridge for image conversion
cv_bridge = CvBridge()

# Variable to store the first frame's timestamp
first_frame_timestamp = None

# Callback for video frames (60 Hz expected)
def video_callback(msg):
    global first_frame_timestamp, first_time_judger
    frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    timestamp = msg.header.stamp.to_sec()

    with buffer_lock:
        if start_time < timestamp:
            video_buffer.append((frame, timestamp))
            if first_time_judger:
                first_frame_timestamp = timestamp
                first_time_judger = False

# Callback for trajectory data (e.g., T265 at 200 Hz)
def trajectory_callback(msg):
    timestamp = msg.header.stamp.to_sec()  # Ensure timestamp is in Unix format (float)
    with buffer_lock:
        if start_time < timestamp:
            pose = msg.pose.pose
            trajectory_buffer.append((timestamp, pose.position.x, pose.position.y, pose.position.z,
                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

# Thread for writing video frames and timestamps
def write_video():
    frame_index = 0
    previous_progress = 0  # Keep track of the last progress value
    pbar = tqdm(total=cfg['episode_len'], desc='Processing Frames')

    while not rospy.is_shutdown():
        with buffer_lock:
            if video_buffer:
                frame, timestamp = video_buffer.popleft()
                video_writer.write(frame)

                # Write timestamp for each frame to CSV
                timestamp_writer.writerow([frame_index, timestamp])
                frame_index += 1

                # Update progress bar
                current_progress = frame_index // 3
                pbar.update(current_progress - previous_progress)
                previous_progress = current_progress

                if frame_index == 3 * cfg['episode_len']:
                    print("Video Done!")
                    pbar.close()  # Close the progress bar
                    break

        sleep(0.001)  # Small sleep to avoid CPU overload


# Thread for writing trajectory data to CSV
def write_trajectory():
    counter = 0
    while not rospy.is_shutdown():
        with buffer_lock:
            if trajectory_buffer:
                Timestamp, PosX, PosY, PosZ, Q_X, Q_Y, Q_Z, Q_W = trajectory_buffer.popleft()
                trajectory_writer.writerow([Timestamp, PosX, PosY, PosZ, Q_X, Q_Y, Q_Z, Q_W])
                counter += 1
                if counter == 10 * cfg['episode_len']:
                    print("Trajectory Done!")
                    break

        sleep(0.001)  # Small sleep to avoid CPU overload

# Main function to start recording
def start_recording():
    global video_subscriber, trajectory_subscriber
    # Subscribe to video and trajectory topics
    video_buffer.clear()
    trajectory_buffer.clear()

    # Start separate threads for writing video and trajectory data
    video_thread = threading.Thread(target=write_video)
    trajectory_thread = threading.Thread(target=write_trajectory)

    video_thread.start()
    trajectory_thread.start()

    video_thread.join()
    trajectory_thread.join()

if __name__ == "__main__":
    # Initialize subscribers
    start_time = 0
    cv_bridge = CvBridge()
    video_subscriber = rospy.Subscriber(config['task_config']['ros']['video_topic'], Image, video_callback, queue_size=config['task_config']['ros']['queue_size'])
    trajectory_subscriber = rospy.Subscriber(config['task_config']['ros']['trajectory_topic'], Odometry, trajectory_callback, queue_size=config['task_config']['ros']['queue_size'])

    # Initialize frame timestamp file
    with open(FRAME_TIMESTAMP_PATH_TEMP, "a", newline='') as frame_timestamp_file:
        frame_timestamp_writer = csv.writer(frame_timestamp_file)
        frame_timestamp_writer.writerow(['Episode Index', 'Timestamp'])

        for episode in range(num_episodes):
            video_writer = cv2.VideoWriter(VIDEO_PATH_TEMP.replace("_n", f"_{episode}"), fourcc, 60, (frame_width, frame_height))

            # CSV for trajectory data and video timestamps
            with open(TRAJECTORY_PATH_TEMP, 'w', newline='') as trajectory_file, \
                 open(TIMESTAMP_PATH_TEMP, 'w', newline='') as timestamp_file:

                trajectory_writer = csv.writer(trajectory_file)
                timestamp_writer = csv.writer(timestamp_file)

                # Write headers to CSV files
                trajectory_writer.writerow(['Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Q_X', 'Q_Y', 'Q_Z', 'Q_W'])
                timestamp_writer.writerow(['Frame Index', 'Timestamp'])

                first_time_judger = False

                input(f"Episode {episode + 1}/{num_episodes} ready. Press Enter to start...")
                start_time = rospy.Time.now().to_sec()  # Start time
                first_time_judger = True
                print(f"Episode {episode + 1}/{num_episodes} started!")

                # Initialize buffers
                obs_replay = []
                action_replay = []

                # Start recording
                try:
                    start_recording()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    raise  # Reraise exception to terminate execution
                finally:
                    frame_timestamp_writer.writerow([episode, first_frame_timestamp])
                    video_writer.release()

                    # Data list preparation
                    data_dict = {
                        '/observations/qpos': [],
                        '/action': [],
                    }
                    for cam_name in cfg['camera_names']:
                        data_dict[f'/observations/images/{cam_name}'] = []
                    timestamp_file.close()
                    timestamps = pd.read_csv(TIMESTAMP_PATH_TEMP)
                    downsampled_timestamps = timestamps.iloc[::3].reset_index(drop=True)
                    cap = cv2.VideoCapture(VIDEO_PATH_TEMP.replace("_n", f"_{episode}"))

                    for idx, row in tqdm(downsampled_timestamps.iterrows(), desc='Extracting Images'):
                        frame_idx = row['Frame Index']
                        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
                        ret, frame = cap.read()
                        if ret:
                            filename = f"{int(frame_idx / 3)}.jpg"
                            cv2.imwrite(os.path.join(IMAGE_PATH, filename), frame)
                            for cam_name in cfg['camera_names']:
                                data_dict[f'/observations/images/{cam_name}'].append(frame)

                    cap.release()

                    # Process trajectory data
                    trajectory = pd.read_csv(TRAJECTORY_PATH_TEMP)
                    trajectory['Timestamp'] = trajectory['Timestamp'].astype(float)

                    for idx, row in tqdm(downsampled_timestamps.iterrows(), desc='Extracting States'):
                        closest_idx = (np.abs(trajectory['Timestamp'] - row['Timestamp'])).argmin()
                        closest_row = trajectory.iloc[closest_idx]
                        pos_quat = [
                            closest_row['Pos X'], closest_row['Pos Y'], closest_row['Pos Z'],
                            closest_row['Q_X'], closest_row['Q_Y'], closest_row['Q_Z'], closest_row['Q_W']
                        ]
                        data_dict['/observations/qpos'].append(pos_quat)
                        data_dict['/action'].append(pos_quat)
                        with open(STATE_PATH, 'a', newline='') as csv_file2:
                            csv_writer2 = csv.writer(csv_file2)
                            csv_writer2.writerow([idx, start_time, closest_row['Timestamp'], row['Timestamp']] + pos_quat)

                    max_timesteps = len(data_dict['/observations/qpos'])
                    idx = len([name for name in os.listdir(data_path) if os.path.isfile(os.path.join(data_path, name))])
                    dataset_path = os.path.join(data_path, f'episode_{idx}.hdf5')
                    os.makedirs(os.path.dirname(dataset_path), exist_ok=True)

                    # Save the data
                    with h5py.File(dataset_path, 'w', rdcc_nbytes=2 * 1024 ** 2) as root:
                        root.attrs['sim'] = False
                        obs = root.create_group('observations')
                        image_grp = obs.create_group('images')
                        for cam_name in cfg['camera_names']:
                            image_grp.create_dataset(
                                cam_name,
                                data=np.array(data_dict[f'/observations/images/{cam_name}'], dtype=np.uint8),
                                compression='gzip',
                                compression_opts=4
                            )
                        root.create_dataset('observations/qpos', data=np.array(data_dict['/observations/qpos']))
                        root.create_dataset('action', data=np.array(data_dict['/action']))

    print("All episodes completed successfully!")