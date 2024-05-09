# you may configure a python virtual environment and a set of paths to automate the extraction of the rosbag
PYTHON_VIRTUAL_ENV='/home/arvc/Applications/venv/bin/python'

# SCANMATCHER outdoor
directory='/media/arvc/INTENSO/DATASETS/INDOOR_OUTDOOR/IO1-2024-05-03-09-51-52'
$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

#directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/O3-2024-03-18-17-11-17'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

#directory='/media/arvc/INTENSO/DATASETS/INDOOR/I3-2024-04-22-15-21-28.bag'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i directory
#$PYTHON_VIRTUAL_ENV run_graphslam_SO3.py -i directory