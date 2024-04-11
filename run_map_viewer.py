"""
    Visualize map from known/ground truth trajectory and LIDAR.

    Author: Arturo Gil.
    Date: 03/2024

    TTD: Save map for future use in MCL localization.
         The map can be now saved in PCL format in order to use o3D directly.
         Also, the map can be saved in a series of pointclouds along with their positions, however path planning using,
         for example, PRM, may not be direct
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
        kf.filter_radius(radii=[1.0, 35.0])
        kf.down_sample()
        Ti = sampled_transforms[i]
        # transform to global and
        pointcloud_temp = kf.transform(T=Ti.array)
        # yuxtaponer los pointclouds
        # pointcloud_global = pointcloud_global + pointcloud_temp
        # vis.add_geometry(pointcloud_global, reset_bounding_box=True)
        vis.add_geometry(pointcloud_temp, reset_bounding_box=True)
        vis.get_render_option().point_size = 1
        # vis.update_geometry(pointcloud_global)
        vis.poll_events()
        vis.update_renderer()
    print('FINISHED! Use the window renderer to observe the map!')
    vis.run()
    vis.destroy_window()


def build_map(global_transforms, keyframemanager, keyframe_sampling=10):
    """
    Caution: in this case, the map is built using a pointcloud and adding the points to it. This may require a great
    amount of memmory
    """
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
        kf.filter_radius(radii=[1.0, 15.0])
        kf.down_sample()
        Ti = sampled_transforms[i]
        # transform to global and
        pointcloud_temp = kf.transform(T=Ti.array)
        # yuxtaponer los pointclouds
        pointcloud_global = pointcloud_global + pointcloud_temp
    print('FINISHED! Use the renderer to view the map')
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
    # TODO: you may be using different estimations to build the map: i.e. scanmatching or the results from graphSLAM
    df_map_poses = euroc_read.read_csv(filename='/robot0/SLAM/scanmatcher_global.csv')
    global_transforms = compute_homogeneous_transforms(df_data=df_map_poses)
    scan_times = df_map_poses['#timestamp [ns]'].to_numpy()
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=voxel_size)

    # Either view the map and visualize or visualize it as it goes
    # Option 1: build the map in a open3D cloud, then render it in a single shot
    # build_map(global_transforms, keyframe_manager, keyframe_sampling=keyframe_sampling)
    # Option 2: use the open3D renderer to add points and view them.
    visualize_map_online(global_transforms, keyframe_manager, keyframe_sampling=keyframe_sampling)


if __name__ == '__main__':
    main()

