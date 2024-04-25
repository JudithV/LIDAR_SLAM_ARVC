import numpy as np
from pyproj import Proj


def gps2utm(df_gps):
    """
    Projects lat, lon to UTM coordinates
    using the origin (first lat, lon)
    """
    latitude = df_gps['latitude']
    longitude = df_gps['longitude']
    altitude = df_gps['altitude']
    # status = df_gps['status']

    # base reference system
    lat_ref = df_gps['latitude'][0]
    lon_ref = df_gps['longitude'][0]
    altitude_ref = df_gps['altitude'][0]

    # status_array = np.array(status)
    myProj = Proj(proj='utm', zone='30', ellps='WGS84', datum='WGS84', preserve_units=False,
                  units='m')

    lat = np.array(latitude)
    lon = np.array(longitude)
    altitude = np.array(altitude)

    UTMx_ref, UTMy_ref = myProj(lon_ref, lat_ref)
    UTMx, UTMy = myProj(lon, lat)
    # UTMx = UTMx[idx]
    # UTMy = UTMy[idx]
    UTMx = UTMx - UTMx_ref
    UTMy = UTMy - UTMy_ref
    altitude = altitude - altitude_ref
    # df_gps.insert(2, 'x', UTMx, True)
    # df_gps.insert(2, 'y', UTMy, True)
    df_gps['x'] = UTMx
    df_gps['y'] = UTMy
    df_gps['altitude'] = altitude
    return df_gps