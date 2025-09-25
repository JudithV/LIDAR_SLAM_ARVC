#EJECUTAR EN LA TERMINAL DE PYCHARM
# python3 extract_rosbag.py

# specify input and ouput files
rosbag_path: "/home/arvc/Documentos/dataset-externos/BLT/rosbag_compressed_2022-06-22-13-02-08.bag"
output_path: "/home/arvc/Documentos/dataset-externos/BLT/2022-06-22-13-02-08"

# Specify the topic name or null to save the data
# ODOMETRY
topic_name_odometry: null #"/husky_velocity_controller/odom"
topic_name_velocity: null #"/husky_velocity_controller/cmd_vel"
# GPS
topic_name_gps: "/gps/fix"  #"/gnss/fix"
topic_name_gps_filtered: "/gps/filtered" # "/gps/filtered"
topic_name_odometry_gps: null # "/odometry/gps"

# LIDAR
topic_name_point_cloud: "/os_cloud_node/points" # "/ouster/points_low_rate"
save_point_cloud_as_pcd: False
save_point_cloud_as_csv: True

# 2D LiDAR
topic_name_laserscan: null # "/scan"

# GROUND TRUTH
topic_name_ground_truth: null # "/ground_truth/state"
topic_name_imu: "/imu/data" #"/imu/data"

# CAMERA
topic_name_camera: null # "/aravis_cam/aravis_cam/image_raw"

# Save transforms if needed
topic_name_tf: null #"/tf"
topic_name_tf_static: null # "/tf_static"
