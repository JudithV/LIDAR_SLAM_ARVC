"""
Simple experiment using GTSAM in a GraphSLAM context.

A series of
"""
from artelib.euler import Euler
from eurocreader.eurocreader import EurocReader
from keyframemanager.keyframemanager import KeyFrameManager
from artelib.homogeneousmatrix import HomogeneousMatrix, compute_global_transformations, \
    compute_relative_transformations
from artelib.quaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from tools.gpsconversions import gps2utm
from tools.sampling import sample_odometry, sample_times
from config import ICP_PARAMETERS
from artelib.homogeneousmatrix import compute_homogeneous_transforms


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



def view_results(relative_transforms_scanmatcher, relative_transforms_odo, df_gps):
    """
    Comparing odo, the scanmatcher result and GPS if available.
    """
    utm_coords_x, utm_coords_y = gps2utm(df_gps)
    delta_x_utm = utm_coords_x[2]-utm_coords_x[0]
    delta_y_utm = utm_coords_y[2]-utm_coords_y[0]
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



def compute_relative_odometry_transformations(df_odo):
    # Now compute absolute and relative transforms for odometry
    # compute homogeneous transforms from odometry
    global_transforms_odo = compute_homogeneous_transforms(df_odo)
    # compute relative homogeneous transforms form global transforms
    relative_transforms_odo = compute_relative_transformations(global_transforms=global_transforms_odo)
    return relative_transforms_odo


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
    return scan_times, odo_times, gps_times, df_odo, df_gps


def scanmatcher():
    """
    The script samples LiDAR data from a starting index. A scanmatching procedure using ICP is carried out.
    The basic parameters to obtain an estimation of the robot movement are:
    - delta_time: the time between LiDAR scans. Beware that, in the ARVC dataset, the initial sample time for LiDARS may be
    as high as 1 second. In this case, a sensible delta_time would be 1s, so as to use all LiDAR data.
    - voxel_size: whether to reduce the pointcloud
    
    Please beware that t
    Sample times from lidar, then find the corresponding data in odometry.
    Transform odometry to relative movements
    """
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    # caution, this is needed to remove initial LiDAR scans with no other data associated to it
    start_index = 15
    # sample LiDAR scans with delta_time in seconds
    delta_time = 0.5
    # voxel size: pointclouds will be filtered with this voxel size
    voxel_size = None
    # select the simple scanmatcher (simple_scanmatcher=True) or the advanced scanmatcher (simple_scanmatcher=False)
    simple_scanmatcher = True
    euroc_read = EurocReader(directory=directory)
    # caution, remove 20 samples (approx.) from the LiDAR data until data capture is stabilized
    scan_times, odo_times, gps_times, df_odo, df_gps = prepare_experiment_data(euroc_read=euroc_read,
                                                                               start_index=start_index,
                                                                               delta_time=delta_time)
    relative_transforms_odo = compute_relative_odometry_transformations(df_odo=df_odo)

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

    # view results in matplotlib. Caution: both global results are modified by T0 using GPS information
    view_results(relative_transforms_scanmatcher, relative_transforms_odo, df_gps)

    # compute the global transformations using scanmatching
    global_transforms_scanmatcher = compute_global_transformations(relative_transforms_scanmatcher, T0=None)

    # save to a directory so that maps can be later built
    # save times for the sensor. Please, beware that, in this sense, there are n-1 times in the relative transformations
    euroc_read.save_sensor_times_as_csv(scan_times, filename='/robot0/scanmatcher/lidar_times.csv')
    # save scanmatcher transforms. Relative
    euroc_read.save_transforms_as_csv(scan_times, relative_transforms_scanmatcher,
                                      filename='/robot0/scanmatcher/scanmatcher_relative.csv')
    # save global transforms
    euroc_read.save_transforms_as_csv(scan_times, global_transforms_scanmatcher,
                                      filename='/robot0/scanmatcher/scanmatcher_global.csv')



if __name__ == "__main__":
    scanmatcher()
