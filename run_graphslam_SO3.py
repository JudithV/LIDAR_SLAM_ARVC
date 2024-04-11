"""
Using GTSAM in a GraphSLAM context.
We are integrating odometry, scanmatching odometry and (if present) GPS.

"""
from graphslam.graphslam import GraphSLAM
from eurocreader.eurocreader import EurocReader
from artelib.homogeneousmatrix import compute_homogeneous_transforms
# from graphslam.dataassociation import DataAssociation



def prepare_experiment_data(euroc_read):
    """
    Read scanmatcher data, and odo.
    Both should be relative transformations
    """
    # Read LiDAR data and sample times
    df_scanmatcher = euroc_read.read_csv(filename='/robot0/mapper/scanmatcher_relative.csv')
    scan_times = df_scanmatcher['#timestamp [ns]'].to_numpy()

    df_odo = euroc_read.read_csv(filename='/robot0/mapper/odo_relative.csv')
    # odo_times = df_odo['#timestamp [ns]'].to_numpy()

    # read and sample gps data
    df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
    gps_times = df_gps['#timestamp [ns]'].to_numpy()
    gps_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=gps_times)
    df_gps = euroc_read.get_df_at_times(df_data=df_gps, time_list=gps_times)
    return scan_times, df_scanmatcher, df_odo, df_gps



def run_graphSLAM():
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    euroc_read = EurocReader(directory=directory)
    # caution, remove 20 samples (approx.) from the LiDAR data until data capture is stabilized
    scan_times, df_scanmatcher, df_odo, df_gps = prepare_experiment_data(euroc_read=euroc_read)

    sm_transforms = compute_homogeneous_transforms(df_scanmatcher)
    odo_transforms = compute_homogeneous_transforms(df_odo)

    # create the graphslam graph
    graphslam = GraphSLAM()
    graphslam.init_graph()

    # create the Data Association object
    # dassoc = DataAssociation(graphslam, delta_index=80, xi2_th=20.0)
    # measured_transforms = []
    # create keyframemanager and add initial observation
    # keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times)
    # keyframe_manager.add_keyframe(0)
    for i in range(len(sm_transforms)):
        # CAUTION: odometry is not used. ICP computed without any prior
        # keyframe_manager.add_keyframe(i)
        # compute relative motion between scan i and scan i-1 0 1, 1 2...
        # atb = keyframe_manager.compute_transformation_local(i-1, i)
        atb = sm_transforms[i]
        # consecutive edges. Adds a new node AND EDGE with restriction aTb
        graphslam.add_consecutive_observation(atb)
        # graphslam.optimize()
        # if (i % 100) == 0:
        #     graphslam.report_on_progress()
        # graphslam.add_relative_observation(atb, i, i+1)
        # graphslam.add_gps_factor(atb, i, i + 1)

        # add scanmatcher
        # graphslam.add_consecutive_observation(atb)
        # graphslam.add_consecutive_observation(atb)

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

        # graphslam.view_solution()
        # or optimizing at every new observation
        # graphslam.optimize()
    graphslam.optimize()
    graphslam.report_on_progress()

    # graphslam.view_solution()
    # keyframe_manager.save_solution(graphslam.get_solution())
    # keyframe_manager.view_map(xgt=odo_gt, sampling=10)
    # or optimizing when all information is available
    # graphslam.optimize()


if __name__ == "__main__":
    run_graphSLAM()
