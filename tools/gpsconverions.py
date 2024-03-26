import numpy as np
from pyproj import Proj


def gps2utm(df_gps):
    latitude = df_gps['latitude']
    longitude = df_gps['longitude']
    status = df_gps['status']
    # base reference system
    lat_ref = df_gps['latitude'][0]
    lon_ref = df_gps['longitude'][0]

    # status_array = np.array(status)
    myProj = Proj(proj='utm', zone='30', ellps='WGS84', datum='WGS84', preserve_units=False,
                  units='m')

    lat = np.array(latitude)
    lon = np.array(longitude)

    UTMx_ref, UTMy_ref = myProj(lon_ref, lat_ref)
    UTMx, UTMy = myProj(lon, lat)
    # UTMx = UTMx[idx]
    # UTMy = UTMy[idx]

    UTMx = UTMx - UTMx_ref
    UTMy = UTMy - UTMy_ref

    return UTMx, UTMy