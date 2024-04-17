"""
Using GTSAM in a GraphSLAM context.
We are integrating odometry, scanmatching odometry and (if present) GPS.

"""
from graphslam.graphSLAMSO3 import GraphSLAMSO3
# from graphslam.graphslam import GraphSLAM
from eurocreader.eurocreader import EurocReader
from artelib.homogeneousmatrix import compute_homogeneous_transforms
# from graphslam.dataassociation import DataAssociation
from keyframemanager.keyframemanager import KeyFrameManager


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
    df_odo = euroc_read.read_csv(filename='/robot0/scanmatcher/odo_relative.csv')
    # odo_times = df_odo['#timestamp [ns]'].to_numpy()

    # read and sample gps data
    df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
    gps_times = df_gps['#timestamp [ns]'].to_numpy()
    gps_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=gps_times)
    df_gps = euroc_read.get_df_at_times(df_data=df_gps, time_list=gps_times)
    return scan_times, df_scanmatcher, df_odo, df_gps

def print_graph_estimates(graphslam):
    print(30 * '*')
    print('INITIAL ESTIMATE')
    print(graphslam.initial_estimate)
    print(30 * '*')
    print('CURRENT ESTIMATE')
    print(graphslam.current_estimate)

def view_result_map(global_transforms, directory, scan_times):
    # use, for example, 1 out of 5 LiDARS to build the map
    keyframe_sampling = 20
    # use, for example, voxel_size=0.2. Use voxel_size=None to use full resolution
    voxel_size = None
    keyframemanager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=voxel_size)
    # OPTIONAL: visualize resulting map
    keyframemanager.add_keyframes(keyframe_sampling=keyframe_sampling)
    # keyframemanager.visualize_map_online(global_transforms=global_transforms, keyframe_sampling=keyframe_sampling)
    keyframemanager.build_map(global_transforms=global_transforms, keyframe_sampling=keyframe_sampling)


def run_graphSLAM():
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    euroc_read = EurocReader(directory=directory)
    # caution, remove 20 samples (approx.) from the LiDAR data until data capture is stabilized
    scan_times, df_scanmatcher, df_odo, df_gps = prepare_experiment_data(euroc_read=euroc_read)

    sm_transforms = compute_homogeneous_transforms(df_scanmatcher)
    odo_transforms = compute_homogeneous_transforms(df_odo)

    # create the graphslam graph
    graphslam = GraphSLAMSO3()
    graphslam.init_graph()

    # print_graph_estimates(graphslam)
    # only perform DA and optimization each 100 poses
    skip_DA_optimization = 100


    # create the Data Association object
    # dassoc = DataAssociation(graphslam, delta_index=80, xi2_th=20.0)
    # measured_transforms = []
    # create keyframemanager and add initial observation
    # keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times)
    # keyframe_manager.add_keyframe(0)
    # start adding scanmatcher info
    for i in range(len(sm_transforms)):
        atb_sm = sm_transforms[i]
        atb_odo = odo_transforms[i]

        # add edge observations between vertices
        graphslam.add_edge(atb_sm, i, i + 1, 'SM')
        graphslam.add_initial_estimate(atb_sm, i)

        # add extra relations between nodes (vertices)
        # graphslam.add_edge(atb_odo, i, i + 1, 'ODO')
        # add extra GPS factors at i

        if (i % skip_DA_optimization) == 0:
            # TODO
            # Data association. Now, add, each cada 15, 20, observaciones (i.e.) 5 metros, ejecutar una asociación de datos
            # Determine if there is loop closure based on the odometry measurement and the previous estimate of the state.
            # j, atb_loop = determine_loop_closure(noisy_tf, current_estimate, i, xyz_tol=18, rot_tol=30)
            # j = None
            # Add a binary factor in between two existing states if loop closure is detected.
            # Otherwise, add a binary factor between a newly observed state and the previous state.
            # graphslam.add_edge(atb_loop, i, j, 'SM')
            graphslam.optimize()
            graphslam.plot(plot3D=True, plot_uncertainty_ellipse=False, skip=5)
            # graphslam.plot2D(plot_uncertainty_ellipse=False, skip=100)

    graphslam.optimize()
    graphslam.plot(plot3D=True, plot_uncertainty_ellipse=True, skip=5)
    global_transforms = graphslam.get_solution_transforms()
    euroc_read.save_transforms_as_csv(scan_times, global_transforms, filename='/robot0/SLAM/solution_graphslam_sm.csv')
    # optional, view resulting map
    view_result_map(global_transforms=global_transforms, directory=directory, scan_times=scan_times)

        # non-consecutive edges
        # associations = dassoc.perform_data_association()
        # for assoc in associations:
        #     # graphslam.view_solution()
        #     i = assoc[0]
        #     j = assoc[1]
        #     atb = keyframe_manager.compute_transformation_global(i, j)
        #     graphslam.add_non_consecutive_observation(i, j, atb)
        #     measured_transforms.append(atb)
        #     # keyframe_manager.view_map()
        # if len(associations):
        #     # graphslam.view_solution()
        #     # optimizing whenever non_consecutive observations are performed (loop closing)
        #     graphslam.optimize()
        #     # graphslam.view_solution()
        #     keyframe_manager.save_solution(graphslam.get_solution())
        #     # keyframe_manager.view_map(xgt=odo_gt)



if __name__ == "__main__":
    run_graphSLAM()
