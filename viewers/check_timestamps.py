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

    plot_delta_times(gps_times, title='GPS delta_time (s)')
    plot_delta_times(lidar_times, title='LIDAR delta_time (s)')
    plot_delta_times(odo_times, title='ODO delta_time (s)')
    plot_delta_times(imu_times, title='IMU delta_time (orientation, s)')


if __name__ == "__main__":
    main()
