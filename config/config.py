import yaml
import pandas as pd
import numpy as np

#
# class Exp_parameters():
#     def __init__(self, yaml_file='config/exp_parameters.yaml'):
#         with open(yaml_file) as file:
#             config = yaml.load(file, Loader=yaml.FullLoader)
#             self.directory = config.get('directory')
#             self.deltaxy = config.get('experiment').get('deltaxy')
#             self.deltath = config.get('experiment').get('deltath')
#             lat, lon = self.calculate_origin_gps()
#             self.origin_lat = lat  # config.get('origin_coordinates').get('lat')
#             self.origin_lon = lon  # config.get('origin_coordinates').get('lon')
#             self.status = config.get('gps').get('reference_status')
#     def calculate_origin_gps(self):
#         gps_csv_filename = self.directory + '/robot0/gps0/data.csv'
#         df_gps = pd.read_csv(gps_csv_filename)
#         lat = np.mean(df_gps['latitude'][0:1])
#         lon = np.mean(df_gps['longitude'][0:1])
#         return lat, lon


class Icp_parameters():
    def __init__(self, yaml_file = 'config/icp_parameters.yaml'):

        with open(yaml_file) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            self.max_distance = config.get('filter_by_distance').get('max_distance')
            self.min_distance = config.get('filter_by_distance').get('min_distance')
            self.voxel_size = config.get('down_sample').get('voxel_size')

            self.radius_gd = config.get('filter_ground_plane').get('radius_normals')
            self.max_nn_gd = config.get('filter_ground_plane').get('maximum_neighbors')

            self.radius_normals = config.get('normals').get('radius_normals')
            self.max_nn = config.get('normals').get('maximum_neighbors')

            self.distance_threshold = config.get('icp').get('distance_threshold')


# EXP_PARAMETERS = Exp_parameters()
ICP_PARAMETERS = Icp_parameters()