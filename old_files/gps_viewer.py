"""
Visualize the GPS
"""
from eurocreader.eurocreader import EurocReader
from tools.plottools import plot_gps_OSM, plot_gps_points


def main():
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O4-2024-03-20-13-14-41'
    euroc_read = EurocReader(directory=directory)
    df_gps = euroc_read.read_csv(filename='/robot0/gps0/data.csv')
    plot_gps_points(df_gps=df_gps, annotate_index=True)
    plot_gps_points(df_gps=df_gps, annotate_error=True)
    plot_gps_OSM(df_gps=df_gps, expand=0.001)


if __name__ == "__main__":
    main()
