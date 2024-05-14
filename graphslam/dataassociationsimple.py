import numpy as np
import matplotlib.pyplot as plt
# from tools.conversions import mod_2pi
import gtsam
import gtsam.utils.plot as gtsam_plot

from artelib.homogeneousmatrix import HomogeneousMatrix


class DataAssociationSimple():
    def __init__(self, graphslam, distance_backwards=7, radius_threshold=5.0):
        """
        Given the current estimation in graphSLAM. Do this process:
        a) Extract the current position/orientation x (last current estimate)
        b) Extract a set of candidates C that are , at least, at a distance_backwards distance from the current pose.
        c) Find whether there are candidates in C within a radius_threshold of the current pose.
        d) Obtain a relative transformation to some of the candidates.
        """
        self.graphslam = graphslam
        # look for data associations that are delta_index back in time
        self.distance_backwards = distance_backwards
        self.radius_threshold = radius_threshold
        self.positions = None

    def loop_closing_simple(self, current_index, number_of_candidates_DA, keyframe_manager):
        """
        A simple loop closing procedure. Given the current pose and index:
            a) Find a number of past robot poses inside a radius_threshold.
            b) Chose number_of_candidates randomly.
            c) Compute observations using ICP for each pair of candidates.
            d) Add the observations with add_loop_closing_restrictions.
        Of course, sometimes, the measurement found using ICP may be wrong, thus inducing errors in the map.
        """
        # Data association. Now, add, each cada 15, 20, observaciones (i.e.) 5 metros, ejecutar una asociación de datos
        # Determine if there is loop closure based on the odometry measurement and the previous estimate of the state.
        # find a number of candidates within a radius
        candidates = self.find_candidates()
        print(candidates)
        i = current_index
        n = np.min([len(candidates), number_of_candidates_DA])
        # generating random samples without replacement. sample the candidates up to the max length n randomly
        candidates = np.random.choice(candidates, size=n, replace=False)
        # from the randomly sampled candidates (number_of_candidates_DA), obtain relative transformations
        # and add loop closing transformations
        for j in candidates:
            Tij = self.compute_transformations_between_candidates(i=i, j=j, keyframe_manager=keyframe_manager)
            self.add_loop_closing_observation(i=i, j=j, Tij=Tij)
        return

    def loop_closing_triangle(self, current_index, number_of_candidates_DA, keyframe_manager):
        """
        A better loop closing procedure. Given the current pose and index i (current_index):
                a) Find a number of past robot poses inside a radius_threshold.
                b) Chose a candidate j randomly. Find another candidate k. The distance in the indexes in j and k < d_index
                c) Compute observations using ICP for Tij and Tik.
                d) Compute Tij*Tjk*(Tik)^(-1)=I, find the error in position and orientation in I to filter the validity
                   of Tij and Tik. Tjk should be low in uncertainty, since it depends on consecutive observations.
                e) Add the observations with add_loop_closing_restrictions.
        Still, of course, sometimes, the measurement found using ICP may be wrong, in this case, it is less probable that
         both Tij and Tik have errors that can cancel each other. As a result, this is a nice manner to filter out observations.
        """
        # forming triplets (i, j1, j2) so that j1 and j2 are within d >< distance.
        triplets = self.find_feasible_triplets(current_index=current_index)
        if len(triplets) == 0:
            return
        triplet_indexes = range(len(triplets))
        # sample candidate indexes
        n = np.min([len(triplet_indexes), number_of_candidates_DA])
        # find pairs of close candidates randomly
        triplet_indexes_sampled = np.random.choice(triplet_indexes, size=n, replace=False)
        for k in triplet_indexes_sampled:
            print('Checking loop closing triplet: ', triplets[k])
            i = triplets[k][0]
            j1 = triplets[k][1]
            j2 = triplets[k][2]
            Tij1 = self.compute_transformations_between_candidates(i=i, j=j1, keyframe_manager=keyframe_manager)
            Tij2 = self.compute_transformations_between_candidates(i=i, j=j2, keyframe_manager=keyframe_manager)
            Tj1j2 = self.compute_consecutive_transformations(i=j1, j=j2)
            # computing a loop closing t
            I = Tij1*Tj1j2*Tij2.inv()
            print('Found loop closing triplet I: ', I)
            if self.check_distances(I):
                 self.add_loop_closing_observation(i=i, j=j1, Tij=Tij1)
                 self.add_loop_closing_observation(i=i, j=j2, Tij=Tij2)


    def find_feasible_triplets(self, current_index):
        triplets = []
        candidates = self.find_candidates()
        print('Found candidates within radius distance threshold:')
        print(candidates)
        if len(candidates) == 0:
            return triplets
        i = current_index
        candidates = np.sort(candidates)
        for k in range(len(candidates)):
            # for each j, find
            j1 = candidates[k]
            j2 = self.find_closest_within_di(j1, candidates[k:])
            if j2 is not None:
                triplets.append([i, j1, j2])
        return triplets

    def check_distances(self, I):
        dp = np.linalg.norm(I.pos())
        da1 = np.linalg.norm(I.euler()[0].abg)
        da2 = np.linalg.norm(I.euler()[1].abg)
        da = min([da1, da2])
        print('Found triangle loop closing distances: ', dp, da)
        if dp < 0.05 and da < 0.05:
            print('I is OK')
            return True
        print('FOUND INCONSISTENT LOOP CLOSING TRIPLET: DISCARDING!!!!!!!')
        return False

    def find_closest_within_di(self, j, rest_of_candidates):
        for i in range(len(rest_of_candidates)):
            u = rest_of_candidates[i]
            if 1 < abs(u - j) < 5:
                return u
        return None

    def add_loop_closing_observation(self, i, j, Tij):
        """
        Adds loop closing restrictions from LiDAR scanmatching
        """
        # for k in range(len(candidates)):
        print('Adding loop_closing edge (i, j): ', i, j)
        # atb_loop = rel_transforms[k]
        # j = candidates[k]
        # atb_loop = self.graphslam.T0_gps.inv()*atb_loop*self.graphslam.T0_gps
        # j = check_transformations_between_candidates (in triangles)
        # Add a binary factor in between two existing states if loop closure is detected.
        self.graphslam.add_edge(Tij, i, j, 'SM')


    def compute_transformations_between_candidates(self, i, j, keyframe_manager):
        """
        Try to compute an observation between the scans at steps i and j in the map.
        The computation is performed considering an initial estimation Tij.
        """
        T0_gps = self.graphslam.T0_gps
        # i = current_index
        # CAUTION: do not add the keyframe to the list: it should have been added before
        # keyframe_manager.add_keyframe(i)
        keyframe_manager.load_pointcloud(i)
        keyframe_manager.pre_process(i)
        # for j in candidates:
        # compute initial error-prone estimation
        Ti = HomogeneousMatrix(self.graphslam.current_estimate.atPose3(i).matrix())
        Tj = HomogeneousMatrix(self.graphslam.current_estimate.atPose3(j).matrix())
        # Correct each estimation by the gps transformation
        # caution, we are estimating the GPS position the robot
        Ti = Ti*T0_gps.inv()
        Tj = Tj * T0_gps.inv()
        # compute the relative transformation between lidars
        # this is the initial estimation between the pointclouds
        Tij = Ti.inv() * Tj
        # compute observation using ICP (change the method)
        keyframe_manager.load_pointcloud(j)
        keyframe_manager.pre_process(j)
        # Caution: the transformation Tijsm is computed from Lidar to Lidar reference frames
        Tijsm = keyframe_manager.compute_transformation(i, j, Tij=Tij)
        # compute the transformation considering the T0_gps transform
        Tijsm = T0_gps.inv()*Tijsm*T0_gps
        # relative_transforms_scanmatcher.append(Tijsm)
        return Tijsm


    def compute_consecutive_transformations(self, i, j):
        """
        Try to compute an observation between the scans at steps i and j in the map.
        The computation is performed considering an initial estimation Tij.
        """
        T0_gps = self.graphslam.T0_gps
        # computing relative transformation from the graphslam current solution
        Ti = HomogeneousMatrix(self.graphslam.current_estimate.atPose3(i).matrix())
        Tj = HomogeneousMatrix(self.graphslam.current_estimate.atPose3(j).matrix())
        # Correct each estimation by the gps transformation
        # caution, we are estimating the GPS position the robot
        Ti = Ti*T0_gps.inv()
        Tj = Tj * T0_gps.inv()
        Tij = Ti.inv() * Tj
        return Tij

    def store_positions(self):
        poses = []
        i = 0
        # fill a vector with all positions
        while self.graphslam.current_estimate.exists(i):
            ce = self.graphslam.current_estimate.atPose3(i)
            T = HomogeneousMatrix(ce.matrix())
            poses.append(T.pos())
            i += 1
        self.positions = np.array(poses)

    def find_candidates(self):
        """
        The function to perform a simple data association.
        The computation of uncertainties is not performed.
        """
        self.store_positions()
        index = self.find_index_backwards()
        candidates = self.find_candidates_within_radius(index)
        return candidates

    # def find_index_backwards(self):
    #     """
    #     Return the maximum index to which we can try to perform data associations.
    #     This is represented by a distance
    #     """
    #     current_pose = self.positions[-1, :]
    #     for i in reversed(range(len(self.positions))):
    #         posei = self.positions[i]
    #         d = np.linalg.norm(current_pose-posei)
    #         if d > self.distance_backwards:
    #             return i
    #     return None

    def find_index_backwards(self):
        """
        Return the maximum index to which we can try to perform data associations.
        This is represented by a distance
        """
        # current_pose = self.positions[-1, :]
        d = 0
        for i in reversed(range(len(self.positions)-1)):
            posei = self.positions[i]
            posei1 = self.positions[i+1]
            di = np.linalg.norm(posei1-posei)
            d += di
            if d > self.distance_backwards:
                return i
        return None


    def find_candidates_within_radius(self, index):
        """
        Find the distance of the current_pose to all the past poses.
        Alternatively: sum the relative distances of consecutive poses.
        """
        if index is None:
            return []
        current_pose = self.positions[-1, :]
        # compute distance to all positions up to index
        positions = self.positions[0:index, :]
        d = np.linalg.norm(positions - current_pose, axis=1)
        # print('distances:')
        # print(d)
        candidates = np.where(d < self.radius_threshold)[0]
        return candidates
        # for i in range(len(self.positions)-index):
        #     diff = self.positions-current_pose
        #
        #     posei = self.positions[i]
        #     d = np.linalg.norm(current_pose-posei)
        #     if d > self.distance_backwards:
        #         return i


    # def marginal_covariance(self, i):
    #     # init initial estimate, read from self.current_solution
    #     initial_estimate = gtsam.Values()
    #     k = 0
    #     for pose2 in self.graphslam.current_solution:
    #         initial_estimate.insert(k, gtsam.Pose2(pose2[0], pose2[1], pose2[2]))
    #         k = k+1
    #     marginals = gtsam.Marginals(self.graphslam.graph, initial_estimate)
    #     cov = marginals.marginalCovariance(i)
    #     return cov
    #
    # def joint_marginal_covariance(self, i, j):
    #     # init initial estimate, read from self.current_solution
    #     initial_estimate = gtsam.Values()
    #     k = 0
    #     for pose2 in self.graphslam.current_solution:
    #         initial_estimate.insert(k, gtsam.Pose2(pose2[0], pose2[1], pose2[2]))
    #         k = k+1
    #     marginals = gtsam.Marginals(self.graphslam.graph, initial_estimate)
    #     keyvector = gtsam.utilities.createKeyVector([i, j])
    #     jm = marginals.jointMarginalCovariance(variables=keyvector).at(iVariable=i, jVariable=j)
    #     return jm
    #
    # def marginal_information(self, i):
    #     # init initial estimate, read from self.current_solution
    #     initial_estimate = gtsam.Values()
    #     k = 0
    #     for pose2 in self.graphslam.current_solution:
    #         initial_estimate.insert(k, gtsam.Pose2(pose2[0], pose2[1], pose2[2]))
    #         k = k+1
    #     marginals = gtsam.Marginals(self.graphslam.graph, initial_estimate)
    #     cov = marginals.marginalInformation(i)
    #     return cov
    #
    # def joint_marginal_information(self, i, j):
    #     # init initial estimate, read from self.current_solution
    #     initial_estimate = gtsam.Values()
    #     k = 0
    #     for pose2 in self.graphslam.current_solution:
    #         initial_estimate.insert(k, gtsam.Pose2(pose2[0], pose2[1], pose2[2]))
    #         k = k+1
    #     marginals = gtsam.Marginals(self.graphslam.graph, initial_estimate)
    #     keyvector = gtsam.utilities.createKeyVector([i, j])
    #     jm = marginals.jointMarginalInformation(variables=keyvector).at(iVariable=i, jVariable=j)
    #     return jm
    #
    # def joint_mahalanobis(self, i, j, only_position=False):
    #     """
    #     Using an approximation for the joint conditional probability of node i and j
    #     """
    #     Oii = self.marginal_information(i)
    #     Ojj = self.marginal_information(j)
    #     Inf_joint = Oii + Ojj
    #     muii = self.graphslam.current_solution[i]
    #     mujj = self.graphslam.current_solution[j]
    #     mu = mujj-muii
    #     mu[2] = mod_2pi(mu[2])
    #     # do not consider orientation
    #     if only_position:
    #         mu[2] = 0.0
    #     d2 = np.abs(np.dot(mu.T, np.dot(Inf_joint, mu)))
    #     return d2
    #
    # def test_conditional_probabilities(self):
    #     """
    #     """
    #     muii = self.graphslam.current_solution[13]
    #     Sii = self.marginal_covariance(13)
    #
    #     Sjj = self.marginal_covariance(1)
    #     mujj = self.graphslam.current_solution[1]
    #
    #     Sij = self.joint_marginal_covariance(1, 13)
    #     Sji = self.joint_marginal_covariance(13, 1)
    #
    #     Sii_ = Sii - np.dot(Sij, np.dot(np.linalg.inv(Sjj), Sij.T))
    #     Sjj_ = Sjj - np.dot(Sij.T, np.dot(np.linalg.inv(Sii), Sij))
    #
    #
    #     # product, joint probability
    #     Sca = np.linalg.inv(np.linalg.inv(Sii) + np.linalg.inv(Sjj))
    #     Scb = Sii + Sjj
    #     a1 = np.dot(np.linalg.inv(Sii), muii)
    #     a2 = np.dot(np.linalg.inv(Sjj), mujj)
    #     mc = np.dot(Sca, a1+a2)
    #
    #     # gtsam_plot.plot_pose2(0, gtsam.Pose2(muii[0], muii[1], muii[2]), 0.5, Sii)
    #     # gtsam_plot.plot_pose2(0, gtsam.Pose2(mujj[0], mujj[1], mujj[2]), 0.5, Sjj)
    #     # gtsam_plot.plot_pose2(0, gtsam.Pose2(0, 0, 0), 0.5, Sii_)
    #     # gtsam_plot.plot_pose2(0, gtsam.Pose2(0, 1.5, 0), 0.5, Sjj_)
    #     # gtsam_plot.plot_pose2(0, gtsam.Pose2(mc[0], mc[1], mc[2]), 0.5, Sca)
    #
    #     for i in range(10):
    #         mu = 0.5*(mujj + muii)
    #         mu[2] = 0
    #         muij = mujj - muii
    #         muij[2] = 0
    #
    #         gtsam_plot.plot_pose2(0, gtsam.Pose2(muii[0], muii[1], muii[2]), 0.5, Sii)
    #         gtsam_plot.plot_pose2(0, gtsam.Pose2(mujj[0], mujj[1], mujj[2]), 0.5, Sjj)
    #         gtsam_plot.plot_pose2(0, gtsam.Pose2(mu[0], mu[1], mu[2]), 0.5, Sca)
    #         gtsam_plot.plot_pose2(0, gtsam.Pose2(mu[0], mu[1], mu[2]), 0.5, Scb)
    #
    #         d0 = np.dot(muij.T, np.dot(np.linalg.inv(Sca), muij))
    #         d1 = np.dot(muij.T, np.dot(np.linalg.inv(Scb), muij))
    #         # d2 = np.dot(muij.T, np.dot(np.linalg.inv(Sii_), muij))
    #         # d3 = np.dot(muij.T, np.dot(np.linalg.inv(Sjj_), muij))
    #
    #         muii += np.array([0.2, 0, 0])
    #     return True
    #
    # def view_full_information_matrix(self):
    #     """
    #     The function i
    #     """
    #     n = self.graphslam.current_index + 1
    #     H = np.zeros((3*n, 3*n))
    #
    #     for i in range(n):
    #         Hii = self.marginal_information(i)
    #         print(i, i)
    #         print(Hii)
    #         H[3 * i:3 * i + 3, 3 * i:3 * i + 3] = Hii
    #
    #     for i in range(n):
    #         for j in range(n):
    #             if i == j:
    #                 continue
    #             Hij = self.joint_marginal_information(i, j)
    #             print(i, j)
    #             print(Hij)
    #             H[3 * i:3 * i + 3, 3 * j:3 * j + 3] = Hij
    #
    #     plt.figure()
    #     plt.matshow(H)
    #     plt.show()
    #     return True
    #
    # def view_full_covariance_matrix(self):
    #     """
    #     """
    #     n = self.graphslam.current_index + 1
    #     H = np.zeros((3*n, 3*n))
    #
    #     for i in range(n):
    #         Hii = self.marginal_covariance(i)
    #         print(i, i)
    #         print(Hii)
    #         H[3 * i:3 * i + 3, 3 * i:3 * i + 3] = Hii
    #
    #     for i in range(n):
    #         for j in range(n):
    #             if i == j:
    #                 continue
    #             Hij = self.joint_marginal_covariance(i, j)
    #             print(i, j)
    #             print(Hij)
    #             H[3 * i:3 * i + 3, 3 * j:3 * j + 3] = Hij
    #
    #     plt.figure()
    #     plt.matshow(H)
    #     plt.show()
    #     return True


