"""
Visualize the all data easily
"""
from eurocreader.eurocreader import EurocReader
from tools.gpsconversions import gps2utm
from tools.plottools import plot_gps_OSM, plot_gps_points, plot_utm_points
from tools.plottools import plot_xy_data, plot_xyz_data, plot_quaternion_data


def view_odo_data(directory):
    euroc_read = EurocReader(directory=directory)
    df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
    plot_xy_data(df_data=df_odo, title='Odometry')


def view_odo_orientation_data(directory):
    """
    Two plots, with odometry and the orientation graph (gamma).
    """
    euroc_read = EurocReader(directory=directory)
    df_odo = euroc_read.read_csv(filename='/robot0/odom/data.csv')
    plot_xy_data(df_data=df_odo, title='Odometry', sample=100, annotate_time=True)

    df_orient = euroc_read.read_csv(filename='/robot0/imu0/orientation/data.csv')
    plot_quaternion_data(df_data=df_orient, title='Quaternion to Euler', annotate_time=True)


def view_gps_data(directory):
    """
    View lat/lng data on 2D. Also, plot on OSM
    """
    try:
        euroc_read = EurocReader(directory=directory)
        df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
        df_gps = gps2utm(df_gps)
        plot_gps_points(df_gps=df_gps, annotate_index=True)
        plot_gps_points(df_gps=df_gps, annotate_error=True)
        plot_utm_points(df_gps=df_gps, annotate_error=True)
        plot_gps_OSM(df_gps=df_gps, save_fig=True, expand=0.001)

    except FileNotFoundError:
        print('NO GPS DATA FOUND')


def view_IMU_data(directory):
    euroc_read = EurocReader(directory=directory)
    df_orient = euroc_read.read_csv(filename='/robot0/imu0/orientation/data.csv')
    plot_quaternion_data(df_data=df_orient, title='Quaternion to Euler')

    df_linear_accel = euroc_read.read_csv(filename='/robot0/imu0/linear_acceleration/data.csv')
    plot_xyz_data(df_data=df_linear_accel, title='Linear Accelerations XYZ')

    df_angular_velocity = euroc_read.read_csv(filename='/robot0/imu0/angular_velocity/data.csv')
    plot_xyz_data(df_data=df_angular_velocity, title='Angular velocities XYZ')


if __name__ == "__main__":
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O5-2024-04-24-12-47-35'
    # uncomment as necessary
    view_IMU_data(directory=directory)
    view_odo_data(directory=directory)
    view_odo_orientation_data(directory=directory)
    view_gps_data(directory=directory)
