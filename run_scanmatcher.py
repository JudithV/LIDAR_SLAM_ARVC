"""
Simple experiment using GTSAM in a GraphSLAM context.

A series of
"""
from artelib.euler import Euler
from eurocreader.eurocreader import EurocReader
from keyframemanager.keyframemanager import KeyFrameManager
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.quaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from tools.sampling import sample_odometry, sample_times
from config import ICP_PARAMETERS



def plot_odometry(df_odo):
    plt.figure()
    plt.scatter(df_odo['x'], df_odo['y'])
    plt.show()


def plot_transformations(transforms):
    positions = []
    orientations = []
    for i in range(len(transforms)):
        t = transforms[i].t2v(n=3)
        positions.append([t[0], t[1], t[2]])
        orientations.append([t[3], t[4], t[5]])

    positions = np.array(positions)
    orientations = np.array(orientations)
    # plt.figure()
    plt.scatter(positions[:, 0], positions[:, 1])
    # plt.show()
    #plot, x, y, z
    # plot alpha, beta, gamma
    # plt.figure()
    # plt.scatter(positions[:, 0], positions[:, 1])
    # plt.show()


def compute_homogeneous_transforms(df_odo):
    """
    Compute homogeneous transforms from global odometry.
    """
    transforms = []
    for i in df_odo.index:
        # CAUTION: THE ORDER IN THE QUATERNION class IS [qw, qx qy qz]
        # the order in ROS is [qx qy qz qw]
        qw = df_odo['qw'][i]
        qx = df_odo['qx'][i]
        qy = df_odo['qy'][i]
        qz = df_odo['qz'][i]
        q = [qw, qx, qy, qz]
        x = df_odo['x'][i]
        y = df_odo['y'][i]
        z = df_odo['z'][i]
        pos = [x, y, z]
        Q = Quaternion(q)
        Ti = HomogeneousMatrix(pos, Q)
        transforms.append(Ti)
    return transforms


def compute_relative_transformations(global_transforms):
    transforms_relative = []
    # compute relative transformations
    for i in range(len(global_transforms) - 1):
        Ti = global_transforms[i]
        Tj = global_transforms[i + 1]
        Tij = Ti.inv() * Tj
        transforms_relative.append(Tij)
    return transforms_relative


def compute_global_transformations(transforms_relative, T0):
    """
    Compute global transformations from relative, starting at T0.
    """
    transforms_global = []
    T = T0
    transforms_global.append(T0)
    # compute global transformations from relative
    for i in range(len(transforms_relative)):
        Tij = transforms_relative[i]
        # Tij.print_nice()
        T = T*Tij
        transforms_global.append(T)
    return transforms_global


def view_results(relative_transforms_scanmatcher, relative_transforms_odo, df_gps):
    """
    Comparing odo, the scanmatcher result and GPS if available.
    """
    utm_coords_x, utm_coords_y = euroc_read.gps2utm(df_gps)


def prepare_experiment_data(euroc_read, start_index=20, delta_time=1.0):
    """
    Read times from LiDAR. Sample times uniformily using delta_time, then obtain the closest data readings at each time
    """
    # Read LiDAR data and sample times
    df_lidar = euroc_read.read_csv(filename='/robot0/lidar/data.csv')
    scan_times = df_lidar['#timestamp [ns]'].to_numpy()
    scan_times = sample_times(sensor_times=scan_times, start_index=start_index, delta_time=delta_time * 1e9)
    # read odometry and get the closest data to each of the times above
    df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
    odo_times = df_odo['#timestamp [ns]'].to_numpy()
    # this finds the data that appear in closest times to the scans (master_sensor_times, sensor_times)
    odo_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=odo_times)
    # now for each time, get the corresponding odometry value
    df_odo = euroc_read.get_df_at_times(df_data=df_odo, time_list=odo_times)
    # read and sample gps data
    df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
    gps_times = df_gps['#timestamp [ns]'].to_numpy()
    gps_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=gps_times)
    df_gps = euroc_read.get_df_at_times(df_data=df_gps, time_list=gps_times)
    # Now compute absolute and relative transforms for odometry
    # compute homogeneous transforms from odometry
    global_transforms_odo = compute_homogeneous_transforms(df_odo)
    # compute relative homogeneous transforms form global transforms
    relative_transforms_odo = compute_relative_transformations(global_transforms=global_transforms_odo)
    return scan_times, odo_times, gps_times, df_gps, global_transforms_odo, relative_transforms_odo


def main():
    """
    Sample times from lidar, then find the corresponding data in odometry.
    Transform odometry to relative movements
    """
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    # caution, this is needed to remove initial LiDAR scans with no other data associated to it
    start_index = 20
    # sample LiDAR scans with delta_time in seconds
    delta_time = 1.5
    # voxel size: pointclouds will be filtered with this voxel size
    voxel_size = 0.05
    # select the simple scanmatcher (simple_scanmatcher=True) or the advanced scanmatcher (simple_scanmatcher=False)
    simple_scanmatcher = True
    euroc_read = EurocReader(directory=directory)
    # caution, remove 20 samples (approx.) from the LiDAR data until data capture is stabilized
    scan_times, odo_times, gps_times, df_gps, global_transforms_odo, relative_transforms_odo = prepare_experiment_data(euroc_read=euroc_read,
                                                                                                   start_index=start_index,
                                                                                                   delta_time=delta_time)




    # keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=ICP_PARAMETERS.voxel_size)
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=voxel_size)
    relative_transforms_scanmatcher = []
    keyframe_manager.add_keyframe(0)
    keyframe_manager.pre_process(0, simple=simple_scanmatcher)
    # now run the scanmatcher routine, for each pair of scans
    for i in range(0, len(scan_times)-1):
        print('Adding keyframe and computing transform: ', i, 'out of ', len(scan_times))
        print('Experiment time is (s): ', (scan_times[i]-scan_times[0])/1e9)
        # add current keyframe
        keyframe_manager.add_keyframe(i+1)
        keyframe_manager.pre_process(i+1, simple=simple_scanmatcher)
        atb_odo = relative_transforms_odo[i]
        print('Initial transform')
        atb_odo.print_nice()
        # usando 2 planes o simple?
        atbsm = keyframe_manager.compute_transformation_local(i, i+1, Tij=atb_odo, simple=simple_scanmatcher)
        relative_transforms_scanmatcher.append(atbsm)
        atbsm.print_nice()

    delta_x_utm = utm_coords_x[1]-utm_coords_x[0]
    delta_y_utm = utm_coords_y[1]-utm_coords_y[0]
    gamma = np.arctan2(delta_y_utm, delta_x_utm)
    T0 = HomogeneousMatrix([0, 0, 0], Euler([0, 0, gamma]))

    # compute global odo transforms from local relative movements and the previous T0
    global_transforms_odo = compute_global_transformations(relative_transforms_odo, T0=T0)
    # compute global transforms from local scanmatcher transforms
    global_transforms_scanmatcher = compute_global_transformations(relative_transforms_scanmatcher, T0=T0)

    plt.figure()
    plot_transformations(global_transforms_odo)
    plot_transformations(global_transforms_scanmatcher)
    plt.plot(utm_coords_x, utm_coords_y)
    plt.show()
    # save scanmatcher transforms
    euroc_read.save_transforms_as_csv(scan_times, relative_transforms_scanmatcher, directory='/robot0/SLAM',
                                      filename='/robot0/SLAM/scanmatcher_relative.csv')
    euroc_read.save_transforms_as_csv(scan_times, relative_transforms_odo, directory='/robot0/SLAM',
                                      filename='/robot0/SLAM/odo_relative.csv')







    # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    #
    # # view map with ground truth transforms
    # keyframe_manager.set_global_transforms(global_transforms=gt_transforms)
    # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    # # equivalent: use relative transforms to compute the global map
    # # keyframe_manager.set_relative_transforms(relative_transforms=gt_transforms_relative)
    # # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    #

if __name__ == "__main__":
    main()
