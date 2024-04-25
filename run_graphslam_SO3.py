"""
Using GTSAM in a GraphSLAM context.
We are integrating odometry, scanmatching odometry and (if present) GPS.

"""
from graphslam.dataassociationsimple import DataAssociationSimple
from graphslam.graphSLAMSO3 import GraphSLAMSO3
# from graphslam.graphslam import GraphSLAM
from eurocreader.eurocreader import EurocReader
from artelib.homogeneousmatrix import compute_homogeneous_transforms, HomogeneousMatrix, \
    compute_relative_transformations
from keyframemanager.keyframemanager import KeyFrameManager
import numpy as np
from tools.gpsconversions import gps2utm
import matplotlib.pyplot as plt

def compute_transformations_between_candidates(graphslam, current_index, candidates, number_of_candidates, keyframe_manager):
    """
    Try to compute an observation between the scans at steps i and j in the map.
    The computation is performed considering an initial estimation Tij.
    """
    # method = 'icppointpoint'
    # method = 'icppointplane'
    method = 'fpfh'
    # method = 'icp2planes'
    i = current_index
    n = np.min([len(candidates), number_of_candidates])
    # generating random samples without replacement
    # sample the candidates up to the max length n randomly
    candidates = np.random.choice(candidates, size=n, replace=False)
    # candidates = candidates[idx]
    relative_transforms_scanmatcher = []
    # CAUTION: do not add the keyframe to the list: it should have been added before
    # keyframe_manager.add_keyframe(i)
    keyframe_manager.load_pointcloud(i)
    keyframe_manager.pre_process(i, method=method)
    for j in candidates:
        # compute initial error-prone estimation
        Ti = HomogeneousMatrix(graphslam.current_estimate.atPose3(i).matrix())
        Tj = HomogeneousMatrix(graphslam.current_estimate.atPose3(j).matrix())
        Tij = Ti.inv()*Tj
        # compute observation using ICP (change the method)
        keyframe_manager.load_pointcloud(j)
        keyframe_manager.pre_process(j, method=method)
        Tijsm = keyframe_manager.compute_transformation(i, j, Tij=Tij, method=method)
        relative_transforms_scanmatcher.append(Tijsm)
    return candidates, relative_transforms_scanmatcher


def prepare_experiment_data(euroc_read):
    """
    Read scanmatcher data, and odo.
    Both should be relative transformations
    """
    # Read LiDAR times
    df_scan_times = euroc_read.read_csv(filename='/robot0/scanmatcher/lidar_times.csv')
    scan_times = df_scan_times['#timestamp [ns]'].to_numpy()
    # Read LiDAR transformation (between each of the previous times)
    df_scanmatcher = euroc_read.read_csv(filename='/robot0/scanmatcher/scanmatcher_relative.csv')
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
        gps_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=gps_times)
        df_gps = euroc_read.get_df_at_times(df_data=df_gps, time_list=gps_times)
        df_gps = gps2utm(df_gps)
    except FileNotFoundError:
        df_gps = None
    return scan_times, df_scanmatcher, df_odo, df_gps


# def print_graph_estimates(graphslam):
#     print(30 * '*')
#     print('INITIAL ESTIMATE')
#     print(graphslam.initial_estimate)
#     print(30 * '*')
#     print('CURRENT ESTIMATE')
#     print(graphslam.current_estimate)


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
    keyframe_manager.visualize_map_online(global_transforms=sampled_global_transforms, radii=[0.5, 10.0])
    # the build map method actually returns a global O3D pointcloud
    pointcloud_global = keyframe_manager.build_map(global_transforms=global_transforms,
                                                   keyframe_sampling=keyframe_sampling, radii=[0.5, 10.0])


def loop_closing(graphslam, dassoc, current_index, number_of_candidates_DA, keyframe_manager):
    # Data association. Now, add, each cada 15, 20, observaciones (i.e.) 5 metros, ejecutar una asociación de datos
    # Determine if there is loop closure based on the odometry measurement and the previous estimate of the state.
    candidates = dassoc.find_candidates()
    if len(candidates) > 0:
        print(candidates)
        candidates, rel_transforms = compute_transformations_between_candidates(graphslam=graphslam, current_index=current_index,
                                                                                candidates=candidates,
                                                                                number_of_candidates=number_of_candidates_DA,
                                                                                keyframe_manager=keyframe_manager)
        graphslam.plot(plot3D=True, plot_uncertainty_ellipse=True, skip=15)
        for k in range(len(candidates)):
            print('Adding loop_closing edge: ', k)
            atb_loop = rel_transforms[k]
            j = candidates[k]
            # j = check_transformations_between_candidates (in triangles)
            # Add a binary factor in between two existing states if loop closure is detected.
            graphslam.add_edge(atb_loop, current_index, j, 'SM')

        graphslam.optimize()
        graphslam.plot(plot3D=True, plot_uncertainty_ellipse=True, skip=15)
        graphslam.plot_simple(skip=5)


def run_graphSLAM():
    # Add the dataset directory
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O2-2024-03-07-13-33-34'
    # Recetario:
    ######################################################################################
    # PARA interiores:
    # el ICP se debe configurar con una altura z máxima y distancia
    # only perform DA and optimization each skip_DA_optimization poses
    perform_DA = True
    skip_DA_optimization = 20
    # try to add at most, number_of_candidates_DA
    number_of_candidates_DA = 5
    # try to find a loop closure, distance_backwards accumulated distance from the last pose
    distance_backwards = 15.0
    # no uncertainty is considered. Trying to close a loop with all candidates poses within this radius
    radius_threshold = 5.0
    # visualization: choose, for example, 1 out of 10 poses and its matching scan
    visualization_keyframe_sampling = 20
    ####################################################################################
    # PARA EXTERIORES
    # radius threshold should be greater
    # if gps is available, then radius threshold can be reduced
    # if
    ##################################################################

    # Build the Euroc format reader
    euroc_read = EurocReader(directory=directory)
    # caution, remove 20 samples (approx.) from the LiDAR data until data capture is stabilized
    # the function gets gps, if available, and computes UTM coordinates at its origin
    scan_times, df_scanmatcher, df_odo, df_gps = prepare_experiment_data(euroc_read=euroc_read)
    sm_transforms = compute_homogeneous_transforms(df_scanmatcher)
    odo_transforms = compute_homogeneous_transforms(df_odo)
    relative_transforms_odo = compute_relative_transformations(global_transforms=odo_transforms)

    # plt.figure()
    # plt.plot(df_gps['altitude'])
    # cov = df_gps['covariance_d3'].tolist()
    # cov = 10000*np.array(cov)
    # plt.plot(cov)
    # plt.show()

    # create the graphslam graph
    graphslam = GraphSLAMSO3()
    graphslam.init_graph()

    # create the Data Association object
    dassoc = DataAssociationSimple(graphslam, distance_backwards=distance_backwards, radius_threshold=radius_threshold)
    print('Adding Keyframes!')
    # create keyframemanager and add initial observation
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=None)
    keyframe_manager.add_keyframes(keyframe_sampling=1)
    # start adding scanmatcher info as edges,
    for i in range(len(sm_transforms)):
        print('GraphSLAM iteration: ', i)
        atb_sm = sm_transforms[i]
        atb_odo = relative_transforms_odo[i]
        # add edge observations between vertices. Adding a binary factor between a newly observed state and the previous state.
        graphslam.add_edge(atb_sm, i, i + 1, 'SM')
        graphslam.add_initial_estimate(atb_sm, i)

        # add extra relations between nodes (ODO vertices)
        graphslam.add_edge(atb_odo, i, i + 1, 'ODO')

        # add extra GPS factors at i
        graphslam.add_GPSfactor(df_gps['x'][i], df_gps['y'][i], df_gps['altitude'][i], i)

        # perform Loop Closing, and optimize
        if perform_DA and ((i % skip_DA_optimization) == 0 or len(sm_transforms)-i < 2):
            graphslam.plot_simple(skip=5)
            loop_closing(graphslam=graphslam, dassoc=dassoc, current_index=i,
                         number_of_candidates_DA=number_of_candidates_DA,
                         keyframe_manager=keyframe_manager)

    graphslam.optimize()
    graphslam.plot(plot3D=True, plot_uncertainty_ellipse=True, skip=15)
    graphslam.plot_simple(skip=1)
    global_transforms = graphslam.get_solution_transforms()
    euroc_read.save_transforms_as_csv(scan_times, global_transforms, filename='/robot0/SLAM/solution_graphslam_sm_gps_odo_0prior.csv')
    # optional, view resulting map
    view_result_map(global_transforms=global_transforms, directory=directory, scan_times=scan_times,
                    keyframe_sampling=visualization_keyframe_sampling)


if __name__ == "__main__":
    run_graphSLAM()
