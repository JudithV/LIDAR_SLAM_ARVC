"""

"""
from __future__ import print_function
import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np


# Declare the 3D translational standard deviations of the prior factor's Gaussian model, in meters.
prior_xyz_sigma = 0.0001
# Declare the 3D rotational standard deviations of the prior factor's Gaussian model, in degrees.
prior_rpy_sigma = 0.0001
# Declare the 3D translational standard deviations of the odometry factor's Gaussian model, in meters.
odo_xyz_sigma = 0.05
# Declare the 3D rotational standard deviations of the odometry factor's Gaussian model, in degrees.
odo_rpy_sigma = 2
# Declare the 3D translational standard deviations of the scanmatcher factor's Gaussian model, in meters.
icp_xyz_sigma = 0.0000000001
# Declare the 3D rotational standard deviations of the odometry factor's Gaussian model, in degrees.
icp_rpy_sigma = 0.0000000001
# GPS noise: in UTM, x, y, height
gps_xyh_sigma = 0.1

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_rpy_sigma*np.pi/180,
                                                         prior_rpy_sigma*np.pi/180,
                                                         prior_rpy_sigma*np.pi/180,
                                                         prior_xyz_sigma,
                                                         prior_xyz_sigma,
                                                         prior_xyz_sigma]))
# noise from the scanmatcher
SM_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([icp_rpy_sigma*np.pi/180,
                                                            icp_rpy_sigma*np.pi/180,
                                                            icp_rpy_sigma*np.pi/180,
                                                            icp_xyz_sigma,
                                                            icp_xyz_sigma,
                                                            icp_xyz_sigma]))

ODO_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([odo_rpy_sigma*np.pi/180,
                                                            odo_rpy_sigma*np.pi/180,
                                                            odo_rpy_sigma*np.pi/180,
                                                            odo_xyz_sigma,
                                                            odo_xyz_sigma,
                                                            odo_xyz_sigma]))

GPS_NOISE = gtsam.Point3(gps_xyh_sigma, gps_xyh_sigma, gps_xyh_sigma)


class GraphSLAM():
    def __init__(self):
        self.current_index = 0
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.current_estimate = self.initial_estimate
        self.n_vertices = 1 # the prior above is an edge
        self.n_edges = 0
        # noises
        self.PRIOR_NOISE = PRIOR_NOISE
        self.SM_NOISE = SM_NOISE
        self.ODO_NOISE = ODO_NOISE
        self.GPS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(GPS_NOISE)

        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(0.1)
        parameters.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(parameters)

    def init_graph(self):
        self.graph.add(gtsam.PriorFactorPose3(self.current_index, gtsam.Pose3(), self.PRIOR_NOISE))
        # CAUTION: the initial T0 transform is the identity.
        self.initial_estimate.insert(self.current_index, gtsam.Pose3())
        self.current_estimate = self.initial_estimate

    def add_consecutive_observation(self, atb):
        """
        aTb is a relative transformation from a to b
        Add a vertex considering two consecutive poses
        """
        self.n_vertices = self.n_vertices + 1
        self.n_edges = self.n_edges + 1
        k = self.current_index
        # add consecutive observation
        self.graph.add(gtsam.BetweenFactorPose3(k, k + 1, gtsam.Pose3(atb.array), self.SM_NOISE))

        # compute next estimation
        next_estimate = self.current_estimate.atPose3(k).compose(gtsam.Pose3(atb.array))
        self.initial_estimate.insert(k + 1, next_estimate)
        self.current_index = k + 1
        # self.isam.update(self.graph, self.initial_estimate)
        # self.current_estimate = self.isam.calculateEstimate()


    def add_non_consecutive_observation(self, i, j, aTb):
        """
        aTb is a relative transformation from frame i to frame j
        Add a vertex considering two consecutive poses
        """
        self.n_edges = self.n_edges + 1
        # add non consecutive observation
        self.graph.add(gtsam.BetweenFactorPose2(int(i), int(j), gtsam.Pose2(aTb[0], aTb[1], aTb[2]), self.ICP_NOISE))


    def optimize(self):
        self.isam.update(self.graph, self.initial_estimate)
        self.current_estimate = self.isam.calculateEstimate()
        self.initial_estimate.clear()
        # if report_on_progress:
        #     self.report_on_progress()

        # # init initial estimate, read from self.current_solution
        # initial_estimate = gtsam.Values()
        # k = 0
        # for c_solution_k in self.current_solution:
        #     initial_estimate.insert(k, gtsam.Pose2(c_solution_k[0], c_solution_k[1], c_solution_k[2]))
        #     k = k+1
        # # solver parameters
        # parameters = gtsam.GaussNewtonParams()
        # # Stop iterating once the change in error between steps is less than this value
        # parameters.setRelativeErrorTol(1e-5)
        # # Do not perform more than N iteration steps
        # parameters.setMaxIterations(100)
        # # Create the optimizer ...
        # optimizer = gtsam.GaussNewtonOptimizer(self.graph, initial_estimate, parameters)
        #
        # # ... and optimize
        # result = optimizer.optimize()
        # print("Final Result:\n{}".format(result))

        # print("GRAPH")
        # print(self.graph)

        # 5. Calculate and print marginal covariances for all variables
        # marginals = gtsam.Marginals(self.graph, result)
        # for i in range(self.n_vertices):
        #     print("X{} covariance:\n{}\n".format(i,
        #                                          marginals.marginalCovariance(i)))
        #
        # for i in range(self.n_vertices):
        #     gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5,
        #                           marginals.marginalCovariance(i))
        # plt.axis('equal')
        # plt.show()

        # now save the solution, caution, the current solution is used as initial
        # estimate in subsequent calls to optimize
        # self.current_solution = []
        # for i in range(self.n_vertices):
        #     x = result.atPose2(i).translation()[0]
        #     y = result.atPose2(i).translation()[1]
        #     th = result.atPose2(i).rotation().theta()
        #     self.current_solution.append(np.array([x, y, th]))
        # self.current_solution = np.array(self.current_solution)

    def report_on_progress(self):
        """Print and plot incremental progress of the robot for 2D Pose SLAM using iSAM2."""

        # Print the current estimates computed using iSAM2.
        print("*" * 50 + f"\nInference after State:\n", self.current_index)
        print(self.current_estimate)

        # Compute the marginals for all states in the graph.
        marginals = gtsam.Marginals(self.graph, self.current_estimate)

        # Plot the newly updated iSAM2 inference.
        fig = plt.figure(0)
        axes = fig.gca(projection='3d')
        plt.cla()

        i = 1
        while self.current_estimate.exists(i):
            gtsam_plot.plot_pose3(0, self.current_estimate.atPose3(i), 0.5,
                                  marginals.marginalCovariance(i))
            i += 1

        axes.set_xlim3d(-30, 45)
        axes.set_ylim3d(-30, 45)
        axes.set_zlim3d(-30, 45)
        plt.pause(.01)


    def view_solution(self):
        # init initial estimate, read from self.current_solution
        initial_estimate = gtsam.Values()
        k = 0
        for pose2 in self.current_solution:
            initial_estimate.insert(k, gtsam.Pose2(pose2[0], pose2[1], pose2[2]))
            k = k+1
        marginals = gtsam.Marginals(self.graph, initial_estimate)
        for i in range(self.n_vertices):
            gtsam_plot.plot_pose2(0, initial_estimate.atPose2(i), 0.5,
                                  marginals.marginalCovariance(i))
        plt.axis('equal')
        plt.show()

    def get_solution(self):
        return self.current_solution


