from __future__ import print_function
import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
from tools.euler import Euler
import numpy as np
import pandas as pd
from config import EXP_PARAMETERS
from tools.homogeneous_matrix import HomogeneousMatrix
from tools.rotationmatrix import RotationMatrix
from pyproj import Proj


class GraphGTSAM3DMIRIAM():
    def __init__(self, scan_times, lat_lon, utm_pos, relative_poses_gps, relative_poses_odo=None,
                 relative_poses_icp=None, relative_poses_imu=None):
        self.scan_times = scan_times
        self.current_index = 0
        self.graph = gtsam.NonlinearFactorGraph()
        self.current_solution = []
        self.n_vertices = 1
        self.n_edges = 0

        self.utm_pos = utm_pos

        self.lat_lon = lat_lon
        self.relative_poses_gps = relative_poses_gps
        self.relative_poses_odo = relative_poses_odo
        self.relative_poses_icp = relative_poses_icp
        self.relative_poses_imu = relative_poses_imu
        self.IMU_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 20, 20, 20]))


        self.init_graph()
        self.initial_estimate = None

        # self.ODO_NOISE = self.calculate_noise(relative_poses_odo)
        self.ODO_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.002])) # [0.1, 0.1, 0.2, 0.2, 0.2, 0.5]))

        # self.ICP_NOISE = self.calculate_noise(relative_poses_icp)
        self.ICP_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.2, 0.2, 0.2, 0.002]))
        self.GPS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(0.1, 0.1, 0.1))

        self.myProj = Proj(proj='utm', zone='30', ellps='WGS84', datum='WGS84', preserve_units=False,
                      units='m')
        self.utmx_ref, self.utmy_ref = self.myProj(EXP_PARAMETERS.origin_lon, EXP_PARAMETERS.origin_lat)


    def init_graph(self):
        # PRIOR NOISE
        # Declare the 3D translational standard deviations of the prior factor's Gaussian model, in meters.
        prior_xyz_sigma = 0.02  # 0.5 # 0.3
        # Declare the 3D rotational standard deviations of the prior factor's Gaussian model, in radians.
        prior_rpy_sigma = 0.02 # 0.5 # 5
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_rpy_sigma, prior_rpy_sigma, prior_rpy_sigma,
                                                                 prior_xyz_sigma, prior_xyz_sigma, prior_xyz_sigma]))
        # pos1 = HomogeneousMatrix([0, 0, 0], Euler([0, 0, 0])).array
        pos0 = self.relative_poses_gps[0].array
        self.current_solution.append(pos0)
        self.graph.add(gtsam.PriorFactorPose3(self.current_index, gtsam.Pose3(pos0), prior_noise))


    def calculate_noise(self, relative_poses_matrix, gps=False):

        if gps == True:
            cov_matrix = np.cov(relative_poses_matrix, rowvar=False)
            cov_i = np.diag(cov_matrix)
            desv = ((np.array(cov_i)) ** 0.5)  # * 4
            NOISE = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(desv[0], desv[1], desv[2]))
            return NOISE

        if relative_poses_matrix == None : return None
        relative_poses = []
        for i in range(1, len(relative_poses_matrix), 1):
            pose_i = HomogeneousMatrix(relative_poses_matrix[i]).t2v()
            relative_poses.append(pose_i)
        cov_matrix = np.cov(relative_poses, rowvar=False)
        cov_i = np.diag(cov_matrix)
        desv = ((np.array(cov_i)) ** 0.5)
        NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.00001, 0.00001, desv[2], desv[0], desv[1], 0.00001]))
        return NOISE

    def add_consecutive_observations(self):

        for i in range(0, len(self.relative_poses_gps)-1, 1):
            self.n_vertices += 1
            self.n_edges += 1
            k = self.current_index

            if self.relative_poses_gps:
                print(f"add gps factor {i}")
                # UTMx, UTMy = self.myProj(self.lat_lon[i][1], self.lat_lon[i][0])
                #
                # utm = gtsam.Point3(UTMx-self.utmx_ref, UTMy-self.utmy_ref, 0.0) #self.lat_lon[i][2])

                utm = gtsam.Point3(self.utm_pos[i][0],self.utm_pos[i][1], 0.0)
                self.graph.add(gtsam.GPSFactor(k, utm, self.GPS_NOISE)) #[0.000310135096; 0.000250469257; 0.36976667];(mapa 2)

            if self.relative_poses_icp:
                print(f"add icp factor {i}")
                matrix_icp = self.relative_poses_icp[i].array
                self.graph.add(gtsam.BetweenFactorPose3(k, k + 1, gtsam.Pose3(matrix_icp),
                                                        self.ICP_NOISE))
            if self.relative_poses_odo:
                print(f'add odo factor {i}')
                matrix_odo = self.relative_poses_odo[i].array
                self.graph.add(gtsam.BetweenFactorPose3(k, k + 1, gtsam.Pose3(matrix_odo),
                                                        self.ODO_NOISE))

            # if self.relative_poses_imu:
            #     print(f"add icp factor {i}")
            #     matrix_imu = self.relative_poses_imu[i].array
            #     self.graph.add(gtsam.BetweenFactorPose3(k, k + 1, gtsam.Pose3(matrix_imu),
            #                                             self.IMU_NOISE))

            #     self.graph.add(gtsam.ImuFactor())

            self.current_index = k+1

    def add_initial_estimate(self):

        for i in range(0, len(self.relative_poses_gps)-1, 1):
            cs = self.current_solution[-1]
            atb = self.relative_poses_gps[i].array
            T = cs
            Trel = atb
            Tn = np.dot(T, Trel)
            self.current_solution.append(Tn)

        self.initial_estimate = gtsam.Values()
        k = 0
        for c_solution_k in self.current_solution:
            self.initial_estimate.insert(k,
                                         gtsam.Pose3(c_solution_k))
            k = k + 1

    def optimize(self):
        parameters = gtsam.LevenbergMarquardtParams()
        parameters.setRelativeErrorTol(1e-5)
        parameters.setMaxIterations(100)
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, parameters)
        result = optimizer.optimize()
        marginals = gtsam.Marginals(self.graph, self.initial_estimate)

        print("Initial Error =", self.graph.error(self.initial_estimate))
        print("Final Error =", self.graph.error(result))

        fig = plt.figure(0)
        axes = fig.add_subplot(projection='3d')
        plt.figure(num=0, figsize=(25, 25))
        for i in range(self.n_vertices):
            gtsam_plot.plot_pose3(0, result.atPose3(i), 4,
                                  marginals.marginalCovariance(i))
            # gtsam_plot.plot_pose3(0, self.initial_estimate.atPose3(i), 4,
            #                       marginals.marginalCovariance(i))
        axes.set_xlim3d(-30, 130)
        axes.set_ylim3d(-30, 130)
        axes.set_zlim3d(-30, 130)
        plt.show()

        corrected_poses = []
        for i in range(self.n_vertices):
            rot = result.atPose3(i).rotation().matrix()
            pos = result.atPose3(i).translation()[0:3]
            pose_i = HomogeneousMatrix(np.array(pos), RotationMatrix(rot)).array
            corrected_poses.append(pose_i)
        corrected_poses = np.array(corrected_poses)
        initial_poses = []
        for i in range(self.n_vertices):
            rot = self.initial_estimate.atPose3(i).rotation().matrix()
            pos = self.initial_estimate.atPose3(i).translation()[0:3]
            pose_i = HomogeneousMatrix(np.array(pos), RotationMatrix(rot)).array
            initial_poses.append(pose_i)
        initial_poses = np.array(initial_poses)

        print(f'current solut \n{self.current_solution[0]}',
              f'\ninitial poses \n{initial_poses[0]}',
              f'\ncorrected poses \n{corrected_poses[0]}',)

        return corrected_poses, initial_poses


'''
class GraphGTSAM2D():
    def __init__(self, scan_times, prior_noise, pos0, relative_poses_gps, relative_poses_odo=None,
                 relative_poses_icp=None):
        self.scan_times = scan_times
        self.current_index = 0
        self.graph = gtsam.NonlinearFactorGraph()
        self.current_solution = np.array([pos0])
        self.graph.add(gtsam.PriorFactorPose2(self.current_index, gtsam.Pose2(pos0[0], pos0[1], pos0[2]), prior_noise))
        self.n_vertices = 1 # the prior above is an edge
        self.n_edges = 0

        self.relative_poses_gps = relative_poses_gps
        self.relative_poses_odo = relative_poses_odo
        self.relative_poses_icp = relative_poses_icp

        self.ODO_NOISE = self.calculate_noise(relative_poses_odo)
        self.ICP_NOISE = self.calculate_noise(relative_poses_icp)
        self.GPS_NOISE = self.calculate_noise(relative_poses_gps, False)
        self.initial_estimate = None

    def calculate_noise(self, relative_poses, gps=False):

        if relative_poses == None : return None

        if gps == True:
            gps_csv_filename = EXP_PARAMETERS.directory + '/robot0/gps0/data.csv'
            df_gps = pd.read_csv(gps_csv_filename)
            NOISE=[]
            for time in self.scan_times:
                ind = df_gps['#timestamp [ns]'].sub(time).abs().idxmin()
                cov_i = [df_gps['covariance_d1'][ind], df_gps['covariance_d2'][ind], df_gps['covariance_d1'][ind]]
                desv = ((np.array(cov_i)) ** 0.5)
                NOISE.append(gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(desv[0], desv[1], desv[2])))
            return np.array(NOISE)

        cov_matrix = np.cov(relative_poses, rowvar=False)
        cov_i = np.diag(cov_matrix)
        desv = ((np.array(cov_i)) ** 0.5) #* 4
        NOISE = gtsam.noiseModel.Diagonal.Sigmas(gtsam.Point3(desv[0], desv[1], desv[2]/10))
        return NOISE

    def add_consecutive_observations(self):

        for i in range(1, len(self.relative_poses_gps), 1):
            self.n_vertices += 1
            self.n_edges += 1
            k = self.current_index

            if self.relative_poses_gps:
                vector_gps = self.relative_poses_gps[i]
                self.graph.add(gtsam.BetweenFactorPose2(k, k + 1,
                                                        gtsam.Pose2(vector_gps[0], vector_gps[1], vector_gps[2]),
                                                        self.GPS_NOISE))
            if self.relative_poses_odo:
                vector_odo = self.relative_poses_odo[i]
                self.graph.add(gtsam.BetweenFactorPose2(k, k + 1, gtsam.Pose2(vector_odo[0], vector_odo[1], vector_odo[2]),
                                                        self.ODO_NOISE))
            if self.relative_poses_icp:
                vector_icp = self.relative_poses_icp[i]
                self.graph.add(gtsam.BetweenFactorPose2(k, k + 1, gtsam.Pose2(vector_icp[0], vector_icp[1], vector_icp[2]),
                                                        self.ICP_NOISE))

            self.current_index = k+1
            cs = self.current_solution[-1]
            atb = self.relative_poses_gps[i]
            T = HomogeneousMatrix([cs[0], cs[1], 0], Euler([0, 0, cs[2]]))
            Trel = HomogeneousMatrix([atb[0], atb[1], 0], Euler([0, 0, atb[2]]))
            T = T * Trel
            self.current_solution = np.vstack((self.current_solution, T.t2v()))

    def add_initial_estimate(self):
        self.initial_estimate = gtsam.Values()
        k = 0
        for c_solution_k in self.current_solution:
            self.initial_estimate.insert(k,
                                         gtsam.Pose2(c_solution_k[0], c_solution_k[1], c_solution_k[2]))
            k = k + 1

    def optimize(self):
        parameters = gtsam.LevenbergMarquardtParams()
        parameters.setRelativeErrorTol(1e-5)
        parameters.setMaxIterations(100)
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, parameters)
        result = optimizer.optimize()
        marginals = gtsam.Marginals(self.graph, self.initial_estimate)

        print("Initial Error =", self.graph.error(self.initial_estimate))
        print("Final Error =", self.graph.error(result))

        plt.figure(num=0, figsize=(25, 25))
        for i in range(self.n_vertices):
            gtsam_plot.plot_pose2(0, self.initial_estimate.atPose2(i), 2.5,
                                  marginals.marginalCovariance(i))

        plt.axis('equal')
        plt.show()
'''
