"""
    Build map from known/ground truth trajectory and LIDAR.
    Author:
"""
from eurocreader.eurocreader import EurocReader
from keyframemanager.keyframemanager import KeyFrameManager
from artelib.homogeneousmatrix import compute_homogeneous_transforms
import open3d as o3d


def visualize_map_online(global_transforms, keyframemanager, keyframe_sampling=10):
    """
    Builds map rendering updates at each frame.

    Caution: the map is not built, but the o3d window is in charge of storing the points
    and viewing them.
    """
    print("COMPUTING MAP FROM KEYFRAMES")
    sampled_transforms = []
    # First: add all keyframes with the known sampling
    for i in range(0, len(keyframemanager.scan_times), keyframe_sampling):
        print("Keyframemanager: Adding Keyframe: ", i, "out of: ", len(keyframemanager.keyframes), end='\r')
        keyframemanager.add_keyframe(i)
        sampled_transforms.append(global_transforms[i])

    print('NOW, BUILD THE MAP')
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # transform all keyframes to global coordinates.
    # pointcloud_global = o3d.geometry.PointCloud()
    # caution, the visualizer only adds the transformed pointcloud to
    # the window, without removing the other geometries
    # the global map (pointcloud_global) is not built.
    for i in range(len(keyframemanager.keyframes)):
        # vis.clear_geometries()
        print("Keyframe: ", i, "out of: ", len(keyframemanager.keyframes), end='\r')
        kf = keyframemanager.keyframes[i]
        kf.filter_radius(radii=[1.0, 5.0])
        kf.down_sample()
        Ti = sampled_transforms[i]
        # transform to global and
        pointcloud_temp = kf.transform(T=Ti.array)
        # yuxtaponer los pointclouds
        # pointcloud_global = pointcloud_global + pointcloud_temp
        # vis.add_geometry(pointcloud_global, reset_bounding_box=True)
        vis.add_geometry(pointcloud_temp, reset_bounding_box=True)
        # vis.update_geometry(pointcloud_global)
        vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()



def build_map(global_transforms, keyframemanager, keyframe_sampling=10):
    print("COMPUTING MAP FROM KEYFRAMES")
    sampled_transforms = []
    # First: add all keyframes with the known sampling
    for i in range(0, len(keyframemanager.scan_times), keyframe_sampling):
        print("Keyframemanager: Adding Keyframe: ", i, "out of: ", len(keyframemanager.keyframes), end='\r')
        keyframemanager.add_keyframe(i)
        sampled_transforms.append(global_transforms[i])

    print('NOW, BUILD THE MAP')
    # transform all keyframes to global coordinates.
    pointcloud_global = o3d.geometry.PointCloud()
    for i in range(len(keyframemanager.keyframes)):
        print("Keyframe: ", i, "out of: ", len(keyframemanager.keyframes), end='\r')
        kf = keyframemanager.keyframes[i]
        kf.filter_radius(radii=[1.0, 5.0])
        kf.down_sample()
        Ti = sampled_transforms[i]
        # transform to global and
        pointcloud_temp = kf.transform(T=Ti.array)
        # yuxtaponer los pointclouds
        pointcloud_global = pointcloud_global + pointcloud_temp
        # draw the whole map
    o3d.visualization.draw_geometries([pointcloud_global])



def main():
    # Read the final transform (i.e. via GraphSLAM)
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    # use, for example, 1 out of 5 LiDARS to build the map
    keyframe_sampling = 1
    # use, for example, voxel_size=0.2. Use None to use full resolution
    voxel_size = None
    euroc_read = EurocReader(directory=directory)
    # test
    df_map_poses = euroc_read.read_csv(filename='/robot0/SLAM/scanmatcher_global.csv')
    global_transforms = compute_homogeneous_transforms(df_data=df_map_poses)
    scan_times = df_map_poses['#timestamp [ns]'].to_numpy()
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=voxel_size)
    build_map(global_transforms, keyframe_manager, keyframe_sampling=keyframe_sampling)
    # visualize_map_online(global_transforms, keyframe_manager, keyframe_sampling=keyframe_sampling)















    # scan_times = keyframe_manager.load_pointclouds()
    #
    # directory_initial_gtsam = (EXP_PARAMETERS.directory + '/robot0/gtsam/initial_gtsam_matrix.csv')
    # directory_results_gtsam = (EXP_PARAMETERS.directory + '/robot0/gtsam/results_gtsam_matrix.csv')
    #
    # poses_initial_gps = read_matrix_csv(directory_initial_gtsam)
    # poses_results = read_matrix_csv(directory_results_gtsam)
    #
    # keyframe_manager.set_global_transforms(poses_results)
    # keyframe_manager.view_map(pcd_directory=EXP_PARAMETERS.directory + '/robot0/gtsam/pcd_results.pcd') #, img_directory = 'Img_map_3' )
    #
    # # BUILD THE MAP ONLY WITH GPS
    # # angles_gps = compute_gps_orientation(utm_pos, 0)
    # gt_poses_gps = compute_homogeneous_transforms(utm_pos, angles_gps)
    # keyframe_manager.set_global_transforms(gt_poses_gps)
    # keyframe_manager.view_map(pcd_directory=EXP_PARAMETERS.directory + '/robot0/gtsam/pcd_gps.pcd')

    # BUILD THE MAP ONLY WITH ODO
    # gt_poses_odo = compute_homogeneous_transforms(odo_pos, odo_orient)
    # keyframe_manager.set_global_transforms(gt_poses_odo)
    # keyframe_manager.view_map(pcd_directory=EXP_PARAMETERS.directory + '/robot0/gtsam/pcd_odo.pcd')


if __name__ == '__main__':
    main()

