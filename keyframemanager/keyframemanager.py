import numpy as np
# import subprocess
from artelib.homogeneousmatrix import HomogeneousMatrix
import open3d as o3d
from keyframemanager.keyframe import KeyFrame


class KeyFrameManager():
    def __init__(self, directory, scan_times, voxel_size):
        """
        given a list of scan times (ROS times), each pcd is read on demand
        """
        self.directory = directory
        self.scan_times = scan_times
        self.keyframes = []
        self.voxel_size = voxel_size

    def add_keyframe(self, index):
        kf = KeyFrame(directory=self.directory, scan_time=self.scan_times[index],
                      voxel_size=self.voxel_size)
        self.keyframes.append(kf)

    def pre_process(self, index, simple):
        self.keyframes[index].pre_process(simple=simple)

    def draw_keyframe(self, index):
        self.keyframes[index].draw_cloud()

    def visualize_keyframe(self, index):
        self.keyframes[index].visualize_cloud()

    def draw_all_clouds(self, sample=3, max_dist=15, max_height=1.5):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for i in range(0, len(self.scan_times), sample):
            vis.clear_geometries()
            self.add_keyframe(i)
            self.keyframes[-1].filter_max_dist(max_dist=max_dist)
            self.keyframes[-1].filter_max_height(max_height=max_height)
            # view = vis.get_view_control()
            # view.set_up(np.array([1, 0, 0]))
            vis.add_geometry(self.keyframes[-1].pointcloud, reset_bounding_box=True)
            # vis.update_geometry(self.keyframes[i].pointcloud)
            vis.poll_events()
            vis.update_renderer()
        vis.destroy_window()

    # def save_solution(self, x):
    #     for i in range(len(x)):
    #         self.keyframes[i].x = x[i]

    # def set_relative_transforms(self, relative_transforms):
    #     """
    #     Given a set of relative transforms. Assign to each keyframe a global transform by
    #     postmultiplication.
    #     Caution, computing global transforms from relative transforms starting from T0=I
    #     """
    #     T = HomogeneousMatrix(np.eye(4))
    #     global_transforms = [T]
    #     for i in range(len(relative_transforms)):
    #         T = T*relative_transforms[i]
    #         global_transforms.append(T)
    #
    #     for i in range(len(self.keyframes)):
    #         self.keyframes[i].set_global_transform(global_transforms[i])
    #
    # def set_global_transforms(self, global_transforms):
    #     """
    #     Assign the global transformation for each of the keyframes.
    #     """
    #     for i in range(len(self.keyframes)):
    #         self.keyframes[i].set_global_transform(global_transforms[i])

    def compute_transformation_local(self, i, j, Tij, simple=False):
        """
        Compute relative transformation using ICP from keyframe i to keyframe j when j-i = 1.
        An initial estimate is used to compute using icp
        """
        # TODO: Compute inintial transformation from IMU
        if simple:
            transform = self.keyframes[i].local_registration_simple(self.keyframes[j], initial_transform=Tij.array)
        else:
            transform = self.keyframes[i].local_registration_two_planes(self.keyframes[j], initial_transform=Tij.array)
        return transform

    # def compute_transformation_local(self, i, j, initial_transform=Tab, use_initial_transform=True):
    #     """
    #     Compute relative transformation using ICP from keyframe i to keyframe j when j-i = 1.
    #     An initial estimate is used to compute using icp
    #     """
    #     # compute initial transform from odometry
    #     # TODO: Compute inintial transformation from IMU
    #     if use_initial_transform:
    #         # initial estimation
    #         xi = self.keyframes[i].x
    #         xj = self.keyframes[j].x
    #         Ti = HomogeneousMatrix([xi[0], xi[1], 0], Euler([0, 0, xi[2]]))
    #         Tj = HomogeneousMatrix([xj[0], xj[1], 0], Euler([0, 0, xj[2]]))
    #         Tij = Ti.inv() * Tj
    #
    #         # muatb = Tij.t2v()
    #         transform = self.keyframes[i].local_registration(self.keyframes[j], initial_transform=Tij.array)
    #         atb = HomogeneousMatrix(transform.transformation) #.t2v()
    #         return atb
    #     else:
    #         transform = self.keyframes[i].local_registration(self.keyframes[j], initial_transform=np.eye(4))
    #         atb = HomogeneousMatrix(transform.transformation) #.t2v()
    #         return atb

    def compute_transformation_global(self, i, j):
        """
        Compute relative transformation using ICP from keyframe i to keyframe j.
        An initial estimate is used.
        FPFh to align and refine with icp
        """
        atb = self.keyframes[i].global_registration(self.keyframes[j])
        atb = HomogeneousMatrix(atb).t2v()
        return atb

    # def build_map(self, keyframe_sampling=10, point_cloud_sampling=1000):
    #     print("COMPUTING MAP FROM KEYFRAMES")
    #     # transform all keyframes to global coordinates.
    #     pointcloud_global = o3d.geometry.PointCloud()
    #     for i in range(0, len(self.keyframes), keyframe_sampling):
    #         print("Keyframe: ", i, "out of: ", len(self.keyframes), end='\r')
    #         kf = self.keyframes[i]
    #         # transform to global and
    #         pointcloud_temp = kf.transform_to_global(point_cloud_sampling=point_cloud_sampling)
    #         # yuxtaponer los pointclouds
    #         pointcloud_global = pointcloud_global + pointcloud_temp
    #     # draw the whole map
    #     o3d.visualization.draw_geometries([pointcloud_global])

        # # now represent ground truth and solution
        # x = []
        # for kf in self.keyframes:
        #     x.append(kf.x)
        # x = np.array(x)
        #
        # plt.figure()
        # # plot ground truth
        # if xgt is not None:
        #     xgt = np.array(xgt)
        #     plt.plot(xgt[:, 0], xgt[:, 1], color='black', linestyle='dashed', marker='+',
        #              markerfacecolor='black', markersize=10)
        # # plot solution
        # plt.plot(x[:, 0], x[:, 1], color='red', linestyle='dashed', marker='o', markerfacecolor='blue', markersize=10)
        # # plt.scatter(points_global[:, 0], points_global[:, 1], color='blue')
        # plt.show(block=True)










