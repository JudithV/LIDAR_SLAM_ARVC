"""
Visualize delta_t for each sensor.
"""
from eurocreader.eurocreader import EurocReader
import numpy as np
import matplotlib.pyplot as plt


def plot_delta_times(sensor_times, units=1e9, title='TITLE'):
    delta_times = []
    for i in range(len(sensor_times)-1):
        dt = sensor_times[i+1]-sensor_times[i]
        delta_times.append(dt/units)
    delta_times = np.array(delta_times)
    plt.title(title)
    plt.plot(range(len(delta_times)), delta_times)
    plt.show(block=True)


def main():
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    euroc_read = EurocReader(directory=directory)

    # read LiDAR times
    df_lidar = euroc_read.read_csv(filename='/robot0/lidar/data.csv')
    lidar_times = df_lidar['#timestamp [ns]']

    # read odometry times
    df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
    odo_times = df_odo['#timestamp [ns]']

    # read gps times
    df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
    gps_times = df_gps['#timestamp [ns]']

    # read IMU times
    df_imu = euroc_read.read_csv(filename='/robot0/imu0/orientation/data.csv')
    imu_times = df_imu['#timestamp [ns]']

    # PLOTTING RELATIVE DELTA TIMES
    plot_delta_times(gps_times, title='GPS delta_time (s)')
    plot_delta_times(lidar_times, title='LIDAR delta_time (s)')
    plot_delta_times(odo_times, title='ODO delta_time (s)')
    plot_delta_times(imu_times, title='IMU delta_time (orientation, s)')

    odo_times = np.asarray(odo_times)
    lidar_times = np.asarray(lidar_times)
    imu_times = np.asarray(imu_times)
    gps_times = np.asarray(gps_times)

    print(30*'*')
    print('TOTAL MESSAGE TIMES: ')
    print('LiDAR: ', len(lidar_times))
    print('Odometry: ', len(odo_times))
    print('GPS: ', len(gps_times))
    print('IMU: ', len(imu_times))
    print(30*'*')

    print(30 * '*')
    print('TOTAL EPOCH TIMES: ')
    print('LiDAR: ', lidar_times[0], lidar_times[-1])
    print('Odometry: ', odo_times[0], odo_times[-1])
    print('GPS: ', gps_times[0], gps_times[-1])
    print('IMU: ', imu_times[0], imu_times[-1])
    print(30 * '*')

    print(30 * '*')
    print('TOTAL EXPERIMENT TIMES (DURATION, seconds): ')
    print('LiDAR: ', (lidar_times[-1]-lidar_times[0])/1e9)
    print('Odometry: ', (odo_times[-1]-odo_times[0])/1e9)
    print('GPS: ', (gps_times[-1]-gps_times[0])/1e9)
    print('IMU: ', (imu_times[-1]-imu_times[0])/1e9)
    print(30 * '*')


if __name__ == "__main__":
    main()
