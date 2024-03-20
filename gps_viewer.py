"""
Visualize the GPS
"""
from eurocreader.eurocreader import EurocReader
from tools.plotgps import plot_gps_OSM, plot_gps_points


def main():
    # directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/2024-03-06-17-30-39'
    euroc_read = EurocReader(directory=directory)
    df_gps = euroc_read.read_csv(directory='/robot0/gps0/data.csv')
    plot_gps_points(df_gps=df_gps, annotate_index=True)
    plot_gps_points(df_gps=df_gps, annotate_error=True)
    plot_gps_OSM(df_gps=df_gps)


if __name__ == "__main__":
    main()
