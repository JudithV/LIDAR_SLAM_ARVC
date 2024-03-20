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

    def pre_process(self, index):
        self.keyframes[index].pre_process()

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

    def compute_transformation_local(self, i, j, Tij):
        """
        Compute relative transformation using ICP from keyframe i to keyframe j when j-i = 1.
        An initial estimate is used to compute using icp
        """
        # TODO: Compute inintial transformation from IMU
        if Tij is not None:
            transform = self.keyframes[i].local_registration(self.keyframes[j], initial_transform=Tij.array)
        else:
            transform = self.keyframes[i].local_registration(self.keyframes[j], initial_transform=np.eye(4))
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

    def view_map(self, keyframe_sampling=10, point_cloud_sampling=1000):
        print("COMPUTING MAP FROM KEYFRAMES")
        # transform all keyframes to global coordinates.
        pointcloud_global = o3d.geometry.PointCloud()
        for i in range(0, len(self.keyframes), keyframe_sampling):
            print("Keyframe: ", i, "out of: ", len(self.keyframes), end='\r')
            kf = self.keyframes[i]
            # transform to global and
            pointcloud_temp = kf.transform_to_global(point_cloud_sampling=point_cloud_sampling)
            # yuxtaponer los pointclouds
            pointcloud_global = pointcloud_global + pointcloud_temp
        # draw the whole map
        o3d.visualization.draw_geometries([pointcloud_global])

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

#
# class KeyFrame():
#     def __init__(self, directory, scan_time, voxel_size):
#         self.transform = None
#         # voxel sizes
#         self.voxel_size = voxel_size
#         self.voxel_size_normals = 5*self.voxel_size
#         self.voxel_size_fpfh = 5*self.voxel_size
#         self.icp_threshold = 5
#         self.fpfh_threshold = 5
#
#         self.max_radius = ICP_PARAMETERS.max_distance
#         self.min_radius = ICP_PARAMETERS.min_distance
#
#         filename = directory + '/robot0/lidar/data/' + str(scan_time) + '.pcd'
#         self.pointcloud = o3d.io.read_point_cloud(filename)
#         # downsample pointcloud and save to pointcloud in keyframe
#         # self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
#         # calcular las normales a cada punto
#         # self.pointcloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
#         #                                                                       max_nn=30))
#
#         # all points
#         # self.pointcloud = None
#         # self.pointcloud_normalized = None
#         # pcd with a segmented ground plate
#         # self.pointcloud_filtered = None
#         self.pointcloud_ground_plane = None
#         self.pointcloud_non_ground_plane = None
#         # self.pointcloud_training = None
#         # self.pcd_fpfh = None
#         # self.compute_groundplane_once = True
#         self.plane_model = None
#
#         # extraer los Fast Point Feature Histograms
#         # self.pointcloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(self.pointcloud,
#         #                                                                        o3d.geometry.KDTreeSearchParamHybrid(
#         #                                                                            radius=self.voxel_size_fpfh,
#         #                                                                            max_nn=100))
#         # self.draw_cloud()
#
#     def pre_process(self, plane_model=None):
#
#         self.pointcloud_filtered = self.filter_by_radius(self.min_radius, self.max_radius)
#         # self.pointcloud_filtered, ind = self.pointcloud_filtered.remove_radius_outlier(nb_points=3, radius=0.3)
#
#         if self.voxel_downsample_size is not None:
#             self.pointcloud_filtered = self.pointcloud_filtered.voxel_down_sample(voxel_size=self.voxel_downsample_size)
#
#         if plane_model is None:
#             self.plane_model = self.calculate_plane(pcd=self.pointcloud_filtered)
#         else:
#             self.plane_model = plane_model
#
#         pcd_ground_plane, pcd_non_ground_plane = self.segment_plane(self.plane_model, pcd=self.pointcloud_filtered)
#         self.pointcloud_ground_plane = pcd_ground_plane
#
#         self.pointcloud_non_ground_plane = pcd_non_ground_plane
#
#         self.pointcloud_ground_plane.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals_ground_plane,
#                                                  max_nn=ICP_PARAMETERS.max_nn_gd))
#         self.pointcloud_non_ground_plane.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
#                                                  max_nn=ICP_PARAMETERS.max_nn))
#         self.pointcloud_filtered.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
#                                                  max_nn=ICP_PARAMETERS.max_nn))
#
#     def local_registration(self, other, initial_transform):
#         """
#         use icp to compute transformation using an initial estimate.
#         caution, initial_transform is a np array.
#         """
#         print("Apply point-to-plane ICP. Local registration")
#         # print("Using threshold: ", self.icp_threshold)
#         # reg_p2p = o3d.pipelines.registration.registration_icp(
#         #         other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform,
#         #         o3d.pipelines.registration.TransformationEstimationPointToPoint())
#         reg_p2p = o3d.pipelines.registration.registration_icp(
#             other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform,
#             o3d.pipelines.registration.TransformationEstimationPointToPlane())
#         print(reg_p2p)
#         # print("Transformation is:")
#         # print(reg_p2p.transformation)
#         print("")
#         other.draw_registration_result(self, reg_p2p.transformation)
#         return reg_p2p.transformation
#
#     def global_registration(self, other):
#         """
#         perform global registration followed by icp
#         """
#         initial_transform = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
#             other.pointcloud, self.pointcloud, other.pointcloud_fpfh, self.pointcloud_fpfh,
#             o3d.pipelines.registration.FastGlobalRegistrationOption(maximum_correspondence_distance=self.fpfh_threshold))
#         # other.draw_registration_result(self, initial_transform.transformation)
#
#         reg_p2p = o3d.pipelines.registration.registration_icp(
#             other.pointcloud, self.pointcloud, self.icp_threshold, initial_transform.transformation,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint())
#         # other.draw_registration_result(self, reg_p2p.transformation)
#         print(reg_p2p)
#         print("Refined transformation is:")
#         print(reg_p2p.transformation)
#         return reg_p2p.transformation
#
#     def draw_registration_result(self, other, transformation):
#         source_temp = copy.deepcopy(self.pointcloud)
#         target_temp = copy.deepcopy(other.pointcloud)
#         source_temp.paint_uniform_color([1, 0, 0])
#         target_temp.paint_uniform_color([0, 0, 1])
#         source_temp.transform(transformation)
#         o3d.visualization.draw_geometries([source_temp, target_temp],
#                                           zoom=0.4459,
#                                           front=[0.9288, -0.2951, -0.2242],
#                                           lookat=[1.6784, 2.0612, 1.4451],
#                                           up=[-0.3402, -0.9189, -0.1996])
#
#     def draw_cloud(self):
#         # o3d.visualization.draw_geometries([self.pointcloud],
#         #                                   zoom=0.3412,
#         #                                   front=[0.4257, -0.2125, -0.8795],
#         #                                   lookat=[2.6172, 2.0475, 1.532],
#         #                                   up=[-0.0694, -0.9768, 0.2024])
#         o3d.visualization.draw_geometries([self.pointcloud])
#
#     # def visualize_cloud(self, vis):
#     #     vis = o3d.visualization.Visualizer()
#     #     vis.create_window()
#     #     vis.add_geometry(self.pointcloud)
#     #     vis.poll_events()
#     #     vis.update_renderer()
#     #     vis.destroy_window()
#
#     def set_global_transform(self, transform):
#         self.transform = transform
#         return
#
#     def transform_to_global(self, point_cloud_sampling=10):
#         """
#             Use open3d to fast transform to global coordinates.
#             Returns the pointcloud in global coordinates
#         """
#         T = HomogeneousMatrix(self.transform)
#         pointcloud = self.pointcloud.uniform_down_sample(every_k_points=point_cloud_sampling)
#         return pointcloud.transform(T.array)
#
#     def filter_max_dist(self, max_dist=5):
#         points = np.asarray(self.pointcloud.points)
#         d = np.linalg.norm(points, axis=1)
#         index = d < max_dist
#         self.pointcloud.points = o3d.utility.Vector3dVector(points[index, :])
#
#     def filter_max_height(self, max_height=1.0):
#         points = np.asarray(self.pointcloud.points)
#         index = points[:, 2] < max_height
#         self.pointcloud.points = o3d.utility.Vector3dVector(points[index, :])
#
#     # def transform(self, T):
#     #     self.pointcloud.transform(T)
#
#
#     def pre_processv2(self):
#         self.pointcloud_filtered = self.filter_by_radius(self.min_radius, self.max_radius)
#         # self.pointcloud_filtered, ind = self.pointcloud_filtered.remove_radius_outlier(nb_points=3, radius=0.3)
#
#         if self.voxel_downsample_size is not None:
#             self.pointcloud_filtered = self.pointcloud_filtered.voxel_down_sample(voxel_size=self.voxel_downsample_size)
#
#         self.pointcloud_filtered.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size_normals,
#                                                  max_nn=ICP_PARAMETERS.max_nn))
#
#     def filter_by_radius(self, min_radius, max_radius):
#         points = np.asarray(self.pointcloud.points)
#         [x, y, z] = points[:, 0], points[:, 1], points[:, 2]
#         r2 = x ** 2 + y ** 2
#         idx = np.where(r2 < max_radius ** 2) and np.where(r2 > min_radius ** 2)
#         return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[idx]))
#
#     def calculate_plane(self, pcd=None, height=-0.5, thresholdA=0.01):
#         # find a plane by removing some of the points at a given height
#         # this best estimates a ground plane.
#
#         if pcd is None:
#             points = np.asarray(self.pointcloud_filtered.points)
#         else:
#             points = np.asarray(pcd.points)
#
#         idx = points[:, 2] < height
#         pcd_plane = o3d.geometry.PointCloud()
#
#         pcd_plane.points = o3d.utility.Vector3dVector(points[idx])
#
#         plane_model, inliers = pcd_plane.segment_plane(distance_threshold=thresholdA, ransac_n=3,
#                                                        num_iterations=1000)
#         [a, b, c, d] = plane_model
#         print(f"Plane model calculated: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
#
#         # plane_model = [0, 0, 1, 0.69]
#         return plane_model
#
#     def segment_plane(self, plane_model, pcd=None, thresholdB=0.4):
#         """
#         filter roughly the points that may belong to the plane.
#         then estimate the plane with these points.
#         find the distance of the points to the plane and classify
#         """
#         # find a plane by removing some of the points at a given height
#         # this best estimates a ground plane.
#         if pcd is None:
#             points = np.asarray(self.pointcloud_filtered.points)
#         else:
#             points = np.asarray(pcd.points)
#         [a, b, c, d] = plane_model
#
#         dist = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / np.sqrt(a * a + b * b + c * c)
#         condicion = dist < thresholdB
#         inliers_final = np.where(condicion == True)
#         inliers_final = inliers_final[0]
#
#         # now select the final pointclouds
#         plane_cloud = pcd.select_by_index(inliers_final)
#         non_plane_cloud = pcd.select_by_index(inliers_final, invert=True)
#         return plane_cloud, non_plane_cloud
#
#     def icp_corrected_transforms(self, keyframe_j, transformation_initial):
#
#         threshold = ICP_PARAMETERS.distance_threshold
#
#         # POINT TO PLANE ICP
#
#         reg_p2pa = (o3d.pipelines.
#                     registration.registration_icp(keyframe_j.pointcloud_ground_plane,
#                                                   self.pointcloud_ground_plane, threshold, transformation_initial,
#                                                   o3d.pipelines.registration.TransformationEstimationPointToPlane()))
#         reg_p2pb = (o3d.pipelines.
#                     registration.registration_icp(keyframe_j.pointcloud_non_ground_plane,
#                                                   self.pointcloud_non_ground_plane, threshold, transformation_initial,
#                                                   o3d.pipelines.registration.TransformationEstimationPointToPlane()))
#
#         t1 = HomogeneousMatrix(reg_p2pa.transformation).t2v(n=3)
#         t2 = HomogeneousMatrix(reg_p2pb.transformation).t2v(n=3)
#         # build solution using both solutions
#         tx = t2[0]
#         ty = t2[1]
#         tz = t1[2]
#         alpha = t1[3]
#         beta = t1[4]
#         gamma = t2[5]
#         T = HomogeneousMatrix(np.array([tx, ty, tz]), Euler([alpha, beta, gamma]))
#         return T
#
#     def icp_corrected_transformsv2(self, keyframe_j, transformation_initial):
#
#         threshold = ICP_PARAMETERS.distance_threshold
#
#         reg_p2pc = (o3d.pipelines.
#                     registration.registration_icp(keyframe_j.pointcloud_filtered,
#                                                   self.pointcloud_filtered, threshold, transformation_initial,
#                                                   o3d.pipelines.registration.TransformationEstimationPointToPlane()))
#
#         T = HomogeneousMatrix(reg_p2pc.transformation)
#         return T
#
#
#









