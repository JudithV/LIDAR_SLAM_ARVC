"""
Visualize the LIDAR point-clouds in the EUROC directory.
"""
from eurocreader.eurocreader import EurocReader
from keyframemanager.keyframemanager import KeyFrameManager


def main():
    directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O4-2024-03-20-13-14-41'
    euroc_read = EurocReader(directory=directory)
    scan_times = euroc_read.read_csv(filename='/robot0/lidar/data.csv')
    scan_times = scan_times['#timestamp [ns]'].to_numpy()
    # create KeyFrameManager
    keyframe_manager = KeyFrameManager(directory=directory, scan_times=scan_times,
                                       voxel_size=0.2)
    keyframe_manager.draw_all_clouds(sample=5, max_dist=25, max_height=1.5)


if __name__ == "__main__":
    main()
