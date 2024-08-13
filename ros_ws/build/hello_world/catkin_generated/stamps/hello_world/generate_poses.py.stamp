#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseStamped

# Parameters
files_path = "/home/salvatore/statistic/dynamic_rotation/events"
# Such as orb3, semorb3 (semantic ORB-SLAM3), uco, suco (semantic UcoSLAM)
slam_method = "semorb3"
dataset_seq = "dynamic_rotation"  # The dataset sequence running
gt_pose_topic = "/optitrack/davis"
slam_pose_topic = "/ze_vio/T_M_B"  # "/orb_slam3/camera_pose" , "/ze_vio/T_M_B"

# Creating txt files for storing poses
print("Creating txt files for adding SLAM and Ground-Truth poses ...")
gt_pose_file_path = f"{files_path}/gt_pose_{slam_method}_{dataset_seq}.txt"
slam_pose_file_path = f"{files_path}/slam_pose_{slam_method}_{dataset_seq}.txt"

gt_pose_file = open(gt_pose_file_path, "w+")
gt_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")

slam_pose_file = open(slam_pose_file_path, "w+")
slam_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")

# Check what to set for the robot-ID
robot_id = -1
if len(sys.argv) > 1:
    robot_id = sys.argv[1]
if len(sys.argv) > 2:
    gt_pose_topic = sys.argv[2]

def groundtruthPoseCallback(groundtruth_pose_msg):
    # Extract pose data
    time = groundtruth_pose_msg.header.stamp.to_sec()
    tx = groundtruth_pose_msg.pose.position.x
    ty = groundtruth_pose_msg.pose.position.y
    tz = groundtruth_pose_msg.pose.position.z
    rx = groundtruth_pose_msg.pose.orientation.x
    ry = groundtruth_pose_msg.pose.orientation.y
    rz = groundtruth_pose_msg.pose.orientation.z
    rw = groundtruth_pose_msg.pose.orientation.w

    # Write to the Ground-Truth file
    gt_pose_file.write(f"{time} {tx} {ty} {tz} {rx} {ry} {rz} {rw}\n")

def slamPoseCallback(slam_pose_msg):
    # Extract pose data
    time = slam_pose_msg.header.stamp.to_sec()
    odom_x = slam_pose_msg.pose.position.x
    odom_y = slam_pose_msg.pose.position.y
    odom_z = slam_pose_msg.pose.position.z
    odom_rx = slam_pose_msg.pose.orientation.x
    odom_ry = slam_pose_msg.pose.orientation.y
    odom_rz = slam_pose_msg.pose.orientation.z
    odom_rw = slam_pose_msg.pose.orientation.w

    # Write to the SLAM file
    slam_pose_file.write(f"{time} {odom_x} {odom_y} {odom_z} {odom_rx} {odom_ry} {odom_rz} {odom_rw}\n")

def subscribers():
    rospy.init_node("text_file_generator", anonymous=True)
    # Subscriber to the ground-truth topic
    rospy.Subscriber(gt_pose_topic, PoseStamped, groundtruthPoseCallback)
    # Subscriber to the SLAM topic
    rospy.Subscriber(slam_pose_topic, PoseStamped, slamPoseCallback)
    rospy.spin()

if __name__ == "__main__":
    try:
        subscribers()
    finally:
        gt_pose_file.close()
        slam_pose_file.close()
