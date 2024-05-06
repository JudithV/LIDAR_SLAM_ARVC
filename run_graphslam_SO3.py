"""
Using GTSAM in a GraphSLAM context.
We are integrating odometry, scanmatching odometry and (if present) GPS.

"""
from graphslam.dataassociationsimple import DataAssociationSimple
from graphslam.graphSLAMSO3 import GraphSLAMSO3
# from graphslam.graphslam import GraphSLAM
from eurocreader.eurocreader import EurocReader
from artelib.homogeneousmatrix import compute_homogeneous_transforms, HomogeneousMatrix, \
    compute_relative_transformations, multiply_by_transform
from keyframemanager.keyframemanager import KeyFrameManager
import numpy as np
from tools.gpsconversions import gps2utm
import matplotlib.pyplot as plt
from artelib.vector import Vector
from artelib.euler import Euler


def prepare_experiment_data(euroc_read):
    """
    Read scanmatcher data, and odo.
    Both should be relative transformations
    """
    # Read LiDAR times
    df_scan_times = euroc_read.read_csv(filename='/robot0/scanmatcher/lidar_times.csv')
    scan_times = df_scan_times['#timestamp [ns]'].to_numpy()
    # Read LiDAR transformation (between each of the previous times)
    df_scanmatcher_global = euroc_read.read_csv(filename='/robot0/scanmatcher/scanmatcher_global.csv')
    # read and sample odometry data
    try:
        df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
        odo_times = df_odo['#timestamp [ns]'].to_numpy()
        odo_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=odo_times)
        df_odo = euroc_read.get_df_at_times(df_data=df_odo, time_list=odo_times)
    except FileNotFoundError:
        df_odo = None
    # read and sample gps data
    try:
        df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
        gps_times = df_gps['#timestamp [ns]'].to_numpy()
        # gps_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=gps_times)
        # df_gps = euroc_read.get_df_at_times(df_data=df_gps, time_list=gps_times)
        df_gps = gps2utm(df_gps)
        T0gps = euroc_read.read_transform('gps0')
    except FileNotFoundError:
        df_gps = None
        gps_times = None
        T0gps = None
    return scan_times, df_scanmatcher_global, df_odo, df_gps, gps_times, T0gps


def get_current_gps_reading(current_time, gps_times, max_delta_time_s=0.1):
    if gps_times is None:
        return None
    diff_vector = (gps_times-current_time)/1e9
    diff_vector = np.abs(diff_vector)
    i = np.argmin(diff_vector)
    if diff_vector[i] < max_delta_time_s:
        return i
    return None



def view_result_map(global_transforms, directory, scan_times, keyframe_sampling):
    """
    View the map (visualize_map_online) or build it.
    When building it, an open3D kd-tree is obtained, which can be saved to a file (i.e.) a csv file.
    Also, the map can be viewed as a set of poses (i.e. x,y,z, alpha, beta, gamma) at certain timestamps associated to
    a scan reading at that time.
    """
    # use, for example, 1 out of 5 LiDARS to build the map
    # keyframe_sampling = 5
    # sample tran
    sampled_global_transforms = []
    for i in range(0, len(global_transforms), keyframe_sampling):
        sampled_global_transforms.append(global_transforms[i])
    # use, for example, voxel_size=0.2. Use voxel_size=None to use full resolution
    voxel_size = None
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=voxel_size)
    # OPTIONAL: visualize resulting map
    keyframe_manager.add_keyframes(keyframe_sampling=keyframe_sampling)
    keyframe_manager.load_pointclouds()
    # caution: only visualization. All points are kept by the visualization window
    # caution: the global transforms correspond to the scan_times
    keyframe_manager.visualize_map_online(global_transforms=sampled_global_transforms, radii=[0.5, 30.0])
    # the build map method actually returns a global O3D pointcloud
    pointcloud_global = keyframe_manager.build_map(global_transforms=global_transforms,
                                                   keyframe_sampling=keyframe_sampling, radii=[0.5, 10.0])


def run_graphSLAM():
    # Add the dataset directory

    # Recetario:
    ######################################################################################
    # PARA EXTERIORES:
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O1-2024-03-06-17-30-39'
    # directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O2-2024-03-07-13-33-34'
    # directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O3-2024-03-18-17-11-17'
    # directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O4-2024-03-20-13-14-41'
    # directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O5-2024-04-24-12-47-35'
    # probado en O5
    # el ICP se debe configurar con una altura z máxima y distancia
    # only perform DA and optimization each skip_DA_optimization poses
    perform_loop_closing = True
    method = 'icppointplane'
    skip_loop_closing = 100
    skip_optimization = 50
    # try to add at most, number_of_candidates_DA
    number_of_candidates_DA = 5
    # try to find a loop closure, distance_backwards accumulated distance from the last pose
    distance_backwards = 15.0
    # no uncertainty is considered. Trying to close a loop with all candidates poses within this radius (includes the error)
    radius_threshold = 5.0
    # visualization: choose, for example, 1 out of 10 poses and its matching scan
    visualization_keyframe_sampling = 20
    ####################################################################################
    # PARA INTERIORES
    # directory = '/media/arvc/INTENSO/DATASETS/INDOOR/I1-2024-03-06-13-44-09'
    # directory = '/media/arvc/INTENSO/DATASETS/INDOOR/I2-2024-03-06-13-50-58'
    # directory = '/media/arvc/INTENSO/DATASETS/INDOOR/I3-2024-04-22-15-21-28'
    # perform_loop_closing = True
    # method = 'icppointplane'
    # skip_loop_closing = 20
    # skip_optimization = 50
    # try to add at most, number_of_candidates_DA
    # number_of_candidates_DA = 5
    # try to find a loop closure, distance_backwards accumulated distance from the last pose
    # distance_backwards = 7.0
    # no uncertainty is considered. Trying to close a loop with all candidates poses within this radius (includes the error)
    # radius_threshold = 3.0
    # visualization: choose, for example, 1 out of 10 poses and its matching scan
    # visualization_keyframe_sampling = 20
    ##################################################################
    # T0: Define initial transformation (Prior for GraphSLAM)
    T0 = HomogeneousMatrix()
    # Caution: Actually, we are estimating the position and orientation of the GPS at this position at the robot.
    # T0_gps = HomogeneousMatrix(Vector([0.36, 0, 0]), Euler([0, 0, 0]))

    # Build the Euroc format reader
    euroc_read = EurocReader(directory=directory)
    # the function gets gps, if available, and computes UTM coordinates at its origin
    # CAUTION: the df_scanmatcher data is referred to the LIDAR reference system. A T0_gps transform should be used if
    # we need to consider GPS readings
    # caution: the GPS is only considered if the timestamp with scan_times (LiDAR) is below 0.05 seconds
    scan_times, df_scanmatcher_global, df_odo_global, df_gps, gps_times, T0_gps = prepare_experiment_data(euroc_read=euroc_read)
    # Read global transformations from scanmatcher and change the reference system to the GPS (if the GPS is available)
    scanmatcher_global = compute_homogeneous_transforms(df_scanmatcher_global)
    scanmatcher_global_gps = multiply_by_transform(scanmatcher_global, Trel=T0_gps)
    scanmatcher_relative_gps = compute_relative_transformations(global_transforms=scanmatcher_global_gps)

    # read odometry and compute relative transforms
    odo_transforms = compute_homogeneous_transforms(df_odo_global)
    odo_transforms_gps = multiply_by_transform(odo_transforms, Trel=T0_gps)
    relative_transforms_odo = compute_relative_transformations(global_transforms=odo_transforms_gps)

    # create the graphslam graph
    graphslam = GraphSLAMSO3(T0=T0, T0_gps=T0_gps)
    graphslam.init_graph()
    # create the Data Association object
    dassoc = DataAssociationSimple(graphslam, distance_backwards=distance_backwards, radius_threshold=radius_threshold)
    print('Adding Keyframes!')
    # create keyframemanager and add initial observation
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=None, method=method)
    keyframe_manager.add_keyframes(keyframe_sampling=1)
    corr_indexes = []
    # start adding scanmatcher info as edges,
    for i in range(len(scanmatcher_relative_gps)):
        print('GraphSLAM trajectory step: ', i)
        current_time = scan_times[i]
        # add extra GPS factors at i, given current time if gps is found at that time (or close to it)
        gps_index = get_current_gps_reading(current_time, gps_times, max_delta_time_s=0.05)
        if gps_index is not None:
            print('Added GPS estimation at pose i: ', i)
            print('***')
            graphslam.add_GPSfactor(df_gps['x'][gps_index], df_gps['y'][gps_index], df_gps['altitude'][gps_index], i)
            corr_indexes.append([i, gps_index])

        # add binary factors using scanmatcher and odometry
        atb_sm = scanmatcher_relative_gps[i]
        atb_odo = relative_transforms_odo[i]

        # create the initial estimate of node i+1 using SM
        graphslam.add_initial_estimate(atb_sm, i + 1)
        # add edge observations between vertices. Adding a binary factor between a newly observed state and the previous state.
        # scanmatching
        graphslam.add_edge(atb_sm, i, i + 1, 'SM')
        # add extra relations between nodes (ODO vertices)
        graphslam.add_edge(atb_odo, i, i + 1, 'ODO')

        # just in case that gps were added
        if i % skip_optimization == 0:
            graphslam.optimize()
            graphslam.plot_simple(skip=1, plot3D=False)

        # perform Loop Closing
        # if perform_loop_closing and ((i % skip_loop_closing) == 0 or len(sm_transforms)-i < 2):
        if perform_loop_closing and (i % skip_loop_closing) == 0 or (len(scanmatcher_relative_gps)-i) < 5:
            graphslam.plot_simple(skip=1, plot3D=False)
            candidates, rel_transforms = dassoc.loop_closing(current_index=i,
                                                             number_of_candidates_DA=number_of_candidates_DA,
                                                             keyframe_manager=keyframe_manager)
            # add_loop_closing_restrictions(graphslam)
            graphslam.plot_simple(skip=1, plot3D=False)
        # graphslam.plot_simple(skip=10, plot3D=False)

    graphslam.optimize()
    # graphslam.plot(plot3D=True, plot_uncertainty_ellipse=True, skip=15)
    # graphslam.plot(plot3D=False, plot_uncertainty_ellipse=False, skip=1)
    graphslam.plot_simple(skip=1, plot3D=False)

    if gps_times is not None:
        graphslam.plot_compare_GPS(df_gps=df_gps, correspondences=corr_indexes)

    # given the estimations, the position and orientation of the LiDAR is retrieved to ease the computation of the maps
    global_transforms_gps = graphslam.get_solution_transforms()
    global_transforms_lidar = graphslam.get_solution_transforms_lidar()
    euroc_read.save_transforms_as_csv(scan_times, global_transforms_lidar, filename='/robot0/SLAM/solution_graphslam.csv')
    # optional, view resulting map
    view_result_map(global_transforms=global_transforms_lidar, directory=directory, scan_times=scan_times,
                    keyframe_sampling=visualization_keyframe_sampling)


if __name__ == "__main__":
    run_graphSLAM()
