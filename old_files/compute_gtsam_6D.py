"""
Author: Miriam Máximo
Date: 11/2023
"""
import numpy as np
import pandas as pd
from config import EXP_PARAMETERS
from eurocreader.eurocreader import EurocReader
from tools.homogeneous_matrix import HomogeneousMatrix
from tools.quaternion import Quaternion
from tools.euler import Euler
from old_files.gtsamgraph_old.gtsamgraph import GraphGTSAM3D
from tools.matrix_csv import read_matrix_csv, write_matrix_csv


def compute_gps_orientation(gps_pos, scan_idx):
    global_orientations = []
    relative_orientations = []
    # global_orientations.append(0)
    # relative_orientations.append(0)
    for i in range(1, len(gps_pos)):
        pos_i_1 = gps_pos[i - 1][0:2]
        pos_i_0 = gps_pos[0][0:2]
        pos_i = gps_pos[i][0:2]
        d_pos = pos_i - pos_i_1
        d_pos_i = pos_i - pos_i_0
        relative_orient = np.arctan2(d_pos[1], d_pos[0])
        absolute_orient = np.arctan2(d_pos_i[1], d_pos_i[0])
        relative_orientations.append(relative_orient)
        global_orientations.append(absolute_orient)

    relative_orientations.append(relative_orientations[len(relative_orientations) - 1])
    global_orientations.append(relative_orientations[len(global_orientations) - 1])
    return relative_orientations


def compute_homogeneous_transforms(gt_pos, gt_orient):
    transforms = []
    for i in range(len(gt_pos)):
        # CAUTION: THE ORDER IN THE QUATERNION class IS [qw, qx qy qz]
        # the order in ROS is [qx qy qz qw]
        try:
            q = [gt_orient[i][3], gt_orient[i][0], gt_orient[i][1], gt_orient[i][2]]
            Q = Quaternion(q)
            Ti = HomogeneousMatrix(gt_pos[i], Q)
        except:
            Ti = HomogeneousMatrix(gt_pos[i], Euler(np.array([0, 0, gt_orient[i]])))
        transforms.append(Ti)
    return transforms


def compute_transformation_local_3D(gt_poses, pos_i):
    relative_poses = []
    pos0 = HomogeneousMatrix(pos_i)
    relative_poses.append(pos0)
    for i in range(1, len(gt_poses), 1):
        Ti = gt_poses[i-1].array
        Tj = gt_poses[i].array
        Ti_inv= np.linalg.inv(Ti)
        Tij = np.dot(Ti_inv, Tj)
        atb = HomogeneousMatrix(Tij)
        relative_poses.append(atb)
    return relative_poses


def main():
    # LECTURA DE DATOS
    directory = EXP_PARAMETERS.directory
    euroc_read = EurocReader(directory)
    deltaxy = EXP_PARAMETERS.deltaxy
    scan_times, lat_lon, utm_pos, odo_pos, odo_orient = euroc_read.prepare_gps_data(deltaxy)

    # CALCULO ANGULOS GPS
    angles_gps = compute_gps_orientation(utm_pos, 0)



    # CREACIÓN DE MATRICES DE TRANSFORMACIÓN ABSOLUTAS Y RELATIVAS
    gt_poses_gps1 = compute_homogeneous_transforms(utm_pos, angles_gps)

    utm_corrected = []
    tranf = HomogeneousMatrix(np.array([0.4, 0.0, 0.0]), Euler(np.array([0, 0, 0]))).array
    gt_poses_gps =[]
    for i in range(0, len(lat_lon), 1):
        Trel = tranf
        Tn = gt_poses_gps1[i].array
        T = np.dot(Tn, np.linalg.inv(Trel))
        utm_corrected.append([HomogeneousMatrix(T).t2v()[0], HomogeneousMatrix(T).t2v()[1]])
        gt_poses_gps.append(HomogeneousMatrix(T))

        # cs = self.current_solution[-1]
        # atb = self.relative_poses_gps[i].array
        # T = cs
        # Trel = atb
        # Tn = np.dot(T, Trel)
        # self.current_solution.append(Tn)

        # utm_pos.append(np.array[UTMx, UTMy])

    relative_poses_gps = compute_transformation_local_3D(gt_poses_gps, gt_poses_gps[0])

    gt_poses_odo = compute_homogeneous_transforms(odo_pos, odo_orient)
    relative_poses_odo = compute_transformation_local_3D(gt_poses_odo, gt_poses_gps[0])

    try:
        icp_directory = EXP_PARAMETERS.directory + '/robot0/gtsam/matrices_icp_global.csv'
        global_poses_icp = read_matrix_csv(icp_directory)
        relative_poses_icp = compute_transformation_local_3D(global_poses_icp, gt_poses_gps[0])
    except:
        relative_poses_icp = None

    imu_csv_filename = EXP_PARAMETERS.directory + '/robot0/imu0/data.csv'
    df_imu = pd.read_csv(imu_csv_filename)
    df_data = df_imu
    time_list = scan_times
    positions = []
    orientations = []
    corresp_time_list = []
    positions = []

    for timestamp in time_list:
        # find the closest timestamp in df
        ind = df_data['#timestamp [ns]'].sub(timestamp).abs().idxmin()

        try:
            gt_o = [df_data['qx'][ind], df_data['qy'][ind], df_data['qz'][ind], df_data['qw'][ind]]
            q = [gt_o[3], gt_o[0], gt_o[1], gt_o[2]]
            Q = Quaternion(q)
            k = Q.Euler().abg[2]  - np.pi
            orientations.append(k)
            positions.append([0, 0, 0])
        except:
            pass

        corresp_time = df_data['#timestamp [ns]'][ind]
        corresp_time_list.append(corresp_time)

    imu_orient = np.array(orientations)
    imu_pos = np.array(positions)
    gt_poses_imu = compute_homogeneous_transforms(utm_pos, imu_orient)
    relative_poses_imu = compute_transformation_local_3D(gt_poses_imu, gt_poses_gps[0])

    #CREACIÓN DEL GRAFO
    graph = GraphGTSAM3D(scan_times, lat_lon, utm_corrected, relative_poses_gps, relative_poses_odo,
                         relative_poses_icp, relative_poses_imu = relative_poses_imu)
    graph.add_consecutive_observations()
    graph.add_initial_estimate()
    corrected_poses, initial_poses = graph.optimize()

    directory_initial_gtsam = (EXP_PARAMETERS.directory + '/robot0/gtsam/initial_gtsam_matrix.csv')
    directory_results_gtsam = (EXP_PARAMETERS.directory + '/robot0/gtsam/results_gtsam_matrix.csv')
    write_matrix_csv(directory_initial_gtsam, initial_poses)
    write_matrix_csv(directory_results_gtsam, corrected_poses)


if __name__ == '__main__':
    main()

