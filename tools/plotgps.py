"""
Plot GPS data from EUROC dir
"""
import matplotlib.pyplot as plt
import tilemapbase
import pandas as pd


def compute_distance(lat1, lng1, lat2, lng2):
    from math import sin, cos, sqrt, atan2, radians
    # approximate radius of earth in m
    R = 6373.0*1000.0
    lat1 = radians(lat1)
    lng1 = radians(lng1)
    lat2 = radians(lat2)
    lng2 = radians(lng2)

    dlon = lng2 - lng1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance


def plot_gps_points(df_gps, title='TITLE'):
    """
    df_gt_gps: ground truth
    df_trajectory_gps: the trajectory
    :param df_gt_gps:
    :param zoom:
    :param color:
    :return:
    """
    plt.figure()
    plt.scatter(x=df_gps['longitude'], y=df_gps['latitude'])
    plt.title(title)
    plt.xlabel('longitude')
    plt.ylabel('latitude')
    for i in range(0, len(df_gps['longitude']), 10):
        txt = str(i)
        plt.annotate(txt, (df_gps['longitude'][i], df_gps['latitude'][i]), fontsize=12)
    plt.show(block=True)


def plot_gps_OSM(df_gps, expand=0.0001, save_fig=False):
    tilemapbase.init(create=True)
    # plt.scatter(df_gps['longitude'], df_gps['latitude'])
    # expand = 0.0001
    extent = tilemapbase.Extent.from_lonlat(
        df_gps.longitude.min() - expand,
        df_gps.longitude.max() + expand,
        df_gps.latitude.min() - expand,
        df_gps.latitude.max() + expand,
    )
    trip_projected = df_gps.apply(
        lambda x: tilemapbase.project(x.longitude, x.latitude), axis=1
    ).apply(pd.Series)
    trip_projected.columns = ["x", "y"]

    tiles = tilemapbase.tiles.build_OSM()
    fig, ax = plt.subplots(figsize=(8, 8), dpi=300)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    plotter = tilemapbase.Plotter(extent, tiles, height=600)
    plotter.plot(ax, tiles, alpha=0.8)
    ax.plot(trip_projected.x, trip_projected.y, color='blue', linewidth=1)
    # ax.scatter(trip_projected.x, trip_projected.y, color='blue')
    plt.axis('off')
    plt.show(block=True)
    if save_fig:
        fig.savefig('trip.png', bbox_inches='tight',pad_inches=0, dpi=300)
