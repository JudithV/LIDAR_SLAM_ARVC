"""
Visualize the point-clouds in the EUROC directory.
"""
from eurocreader.eurocreader import EurocReader
from graphslam.keyframemanager import KeyFrameManager


def main():
    directory = '/media/arvc/INTENSO/DATASETS/2024-03-06/2024-03-06-13-44-09'
    euroc_read = EurocReader(directory=directory)
    scan_times = euroc_read.read_csv(directory='/robot0/lidar/data.csv')
    scan_times = scan_times['#timestamp [ns]'].to_numpy()
    # create KeyFrameManager
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times,
                                       voxel_size=0.1)
    keyframe_manager.draw_all_clouds(sample=1, max_dist=10, max_height=1.5)


if __name__ == "__main__":
    main()
