"""
Simple experiment using GTSAM in a GraphSLAM context.

A series of
"""
from eurocreader.eurocreader import EurocReader
from keyframemanager.keyframemanager import KeyFrameManager
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.quaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from tools.sampling import sample_odometry, sample_times
from config import ICP_PARAMETERS



    #     plt.plot(range(len(data)), data[:, 0], color='red', linestyle='dashed', marker='o', markersize=12)
    #     plt.plot(range(len(data)), data[:, 1], color='green', linestyle='dashed', marker='o', markersize=12)
    #     plt.plot(range(len(data)), data[:, 2], color='blue', linestyle='dashed', marker='o', markersize=12)
    #     plt.show(block=True)
# def eval_errors(ground_truth_transforms, measured_transforms):
#     # compute xyz alpha beta gamma
#     gt_tijs = []
#     meas_tijs = []
#     for i in range(len(ground_truth_transforms)):
#         gt_tijs.append(ground_truth_transforms[i].t2v(n=3))  # !!! convert to x y z alpha beta gamma
#         meas_tijs.append(measured_transforms[i].t2v(n=3))
#
#     gt_tijs = np.array(gt_tijs)
#     meas_tijs = np.array(meas_tijs)
#     errors = gt_tijs-meas_tijs
#
#     plt.figure()
#     plt.plot(range(len(errors)), errors[:, 0], color='red', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(errors)), errors[:, 1], color='green', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(errors)), errors[:, 2], color='blue', linestyle='dashed', marker='o', markersize=12)
#     plt.title('Errors XYZ')
#     plt.show(block=True)
#
#     plt.figure()
#     plt.plot(range(len(errors)), errors[:, 3], color='red', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(errors)), errors[:, 4], color='green', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(errors)), errors[:, 5], color='blue', linestyle='dashed', marker='o', markersize=12)
#     plt.title('Errors Alfa Beta Gamma')
#     plt.show(block=True)
#
#     print("Covariance matrix: ")
#     print(np.cov(errors.T))
#
#
# def view_pos_data(data):
#     plt.figure()
#     plt.plot(range(len(data)), data[:, 0], color='red', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(data)), data[:, 1], color='green', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(data)), data[:, 2], color='blue', linestyle='dashed', marker='o', markersize=12)
#     plt.show(block=True)
#
#     plt.figure()
#     plt.plot(data[:, 0], data[:, 1], color='blue', linestyle='dashed', marker='o', markersize=12)
#     plt.show(block=True)
#
#
# def view_orient_data(data):
#     eul = []
#     for dat in data:
#         q = [dat[3], dat[0], dat[1], dat[2]]
#         Q = Quaternion(q)
#         th = Q.Euler()
#         eul.append(th.abg)
#     eul = np.array(eul)
#
#     plt.figure()
#     plt.plot(range(len(eul)), eul[:, 0], color='red', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(eul)), eul[:, 1], color='green', linestyle='dashed', marker='o', markersize=12)
#     plt.plot(range(len(eul)), eul[:, 2], color='blue', linestyle='dashed', marker='o', markersize=12)
#     # plt.legend()
#     plt.show(block=True)



def plot_odometry(df_odo):
    plt.figure()
    plt.scatter(df_odo['x'], df_odo['y'])
    plt.show()


def plot_transformations(transforms):
    positions = []
    orientations = []
    for i in range(len(transforms)):
        t = transforms[i].t2v(n=3)
        positions.append([t[0], t[1], t[2]])
        orientations.append([t[3], t[4], t[5]])

    positions = np.array(positions)
    orientations = np.array(orientations)
    plt.figure()
    plt.scatter(positions[:, 0], positions[:, 1])
    plt.show()
    #plot, x, y, z
    # plot alpha, beta, gamma
    # plt.figure()
    # plt.scatter(positions[:, 0], positions[:, 1])
    # plt.show()


def compute_homogeneous_transforms(df_odo):
    transforms = []
    for i in range(len(df_odo)):
        # CAUTION: THE ORDER IN THE QUATERNION class IS [qw, qx qy qz]
        # the order in ROS is [qx qy qz qw]
        qw = df_odo['qw'][i]
        qx = df_odo['qx'][i]
        qy = df_odo['qy'][i]
        qz = df_odo['qz'][i]
        q = [qw, qx, qy, qz]
        x = df_odo['x'][i]
        y = df_odo['y'][i]
        z = df_odo['z'][i]
        pos = [x, y, z]
        Q = Quaternion(q)
        Ti = HomogeneousMatrix(pos, Q)
        transforms.append(Ti)
    return transforms


def compute_relative_transformations(transforms):
    transforms_relative = []
    # compute relative transformations
    for i in range(len(transforms) - 1):
        Ti = transforms[i]
        Tj = transforms[i + 1]
        Tij = Ti.inv() * Tj
        transforms_relative.append(Tij)
    return transforms_relative


def compute_global_transformations(transforms_relative):
    transforms_global = []
    T = HomogeneousMatrix(np.eye(4))
    # compute global transformations from relative
    for i in range(len(transforms_relative)):
        Tij = transforms_relative[i]
        Tij.print_nice()
        T = T*Tij
        transforms_global.append(T)
    return transforms_global

# def prepare_experiment_odo(directory):
#     euroc_read = EurocReader(directory=directory)
#     # read odometry and sample
#     df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
#     odo_times, df_odo = sample_odometry(df_odo=df_odo, deltaxy=0.3, deltath=0.05)
#     df_lidar = euroc_read.read_csv(filename='/robot0/lidar/data.csv')
#     scan_times = df_lidar['#timestamp [ns]'].to_numpy()
#     # this finds the closest times to each odometry data
#     scan_times = euroc_read.get_closest_times(odo_times, scan_times, max_time_dif_s=0.3)
#     # plot_odometry(df_odo=df_odo)
#     # compute homogeneous transforms from odometry
#     transforms = compute_homogeneous_transforms(df_odo)
#     # compute relative homogeneous transforms form global transforms
#     transforms_rel = compute_relative_transformations(transforms=transforms)


def plot_deltas(sensor_times, units=1e9):
    delta_times = []
    for i in range(len(sensor_times)-1):
        dt = sensor_times[i+1]-sensor_times[i]
        delta_times.append(dt/units)
    delta_times = np.array(delta_times)
    plt.plot(range(len(delta_times)), delta_times)
    plt.show()




def main():
    """
    Sample times from lidar, then find the corresponding data in odometry.
    Transform odometry to relative movements
    """
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    euroc_read = EurocReader(directory=directory)
    df_lidar = euroc_read.read_csv(filename='/robot0/lidar/data.csv')
    lidar_times = df_lidar['#timestamp [ns]']
    # plot_deltas(lidar_times)
    scan_times = sample_times(sensor_times=lidar_times, delta_time=0.8*1e9)
    # plot_deltas(scan_times)

    # read odometry and sample
    df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
    odo_times = df_odo['#timestamp [ns]']
    # this finds the data that appear in closest times to the scans (master_sensor_times, sensor_times)
    odo_times = euroc_read.get_closest_times(master_sensor_times=scan_times, sensor_times=odo_times,
                                             max_time_dif_s=0.3*1e9)
    df_odo = df_odo.loc[df_odo['#timestamp [ns]'].isin(odo_times)]

    # plot_deltas(odo_times)

    # now, get the odometry at those times

    # plot_odometry(df_odo=df_odo)
    # plot_data --< xyz,
    # compute homogeneous transforms from odometry
    transforms = compute_homogeneous_transforms(df_odo)
    # compute relative homogeneous transforms form global transforms
    transforms_rel = compute_relative_transformations(transforms=transforms)

    # now run the scanmatcher routine
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times, voxel_size=ICP_PARAMETERS.voxel_size)
    scanmatcher_transforms = []

    # for i in range(0, len(scan_times)-1):
    for i in range(3):
        print('Adding keyframe and computing transform: ', i, 'out of ', len(scan_times))
        print('Experiment time is (s): ', (scan_times[i]-scan_times[0])/1e9)
        keyframe_manager.add_keyframe(i)
        keyframe_manager.pre_process(i)
        keyframe_manager.add_keyframe(i+1)
        keyframe_manager.pre_process(i+1)
        atb_odo = transforms_rel[i]
        atb = keyframe_manager.compute_transformation_local(i, i+1, Tij=atb_odo)
        scanmatcher_transforms.append(atb)

    print(scanmatcher_transforms)
    # compute
    global_scanmatcher_transforms = compute_global_transformations(scanmatcher_transforms)
    plot_transformations(global_scanmatcher_transforms)

    euroc_read.save_transforms_as_csv(scan_times, global_scanmatcher_transforms, filename='/robot0/scanmatcher/data.csv')

    # save transforms

    # compute relative transformations odo
    # get lidars at each time
    # compute relative transformations ICP
    # save to csv at directory/slam/scanmatcher.csv



    # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    #
    # # view map with ground truth transforms
    # keyframe_manager.set_global_transforms(global_transforms=gt_transforms)
    # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    # # equivalent: use relative transforms to compute the global map
    # # keyframe_manager.set_relative_transforms(relative_transforms=gt_transforms_relative)
    # # keyframe_manager.view_map(keyframe_sampling=30, point_cloud_sampling=20)
    #

if __name__ == "__main__":
    main()
