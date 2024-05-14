# you may configure a python virtual environment and a set of paths to automate the extraction of the rosbag
PYTHON_VIRTUAL_ENV='/home/arvc/Applications/venv/bin/python'

# INDOOR
#directory='/media/arvc/INTENSO/DATASETS/INDOOR/I1-2024-03-06-13-44-09'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory
#
#directory='/media/arvc/INTENSO/DATASETS/INDOOR/I2-2024-03-06-13-50-58'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory
#
#directory='/media/arvc/INTENSO/DATASETS/INDOOR/I3-2024-04-22-15-21-28'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory
#
# OUTDOOR
#directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/O1-2024-03-06-17-30-39'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory
#
#directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/O2-2024-03-07-13-33-34'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory
#
#directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/O3-2024-03-18-17-11-17'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

#directory = '/media/arvc/INTENSO/DATASETS/OUTDOOR/O4-2024-03-20-13-14-41'
#$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

directory='/media/arvc/INTENSO/DATASETS/OUTDOOR/O5-2024-04-24-12-47-35'
$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

# MIXED
directory='/media/arvc/INTENSO/DATASETS/INDOOR_OUTDOOR/IO1-2024-05-03-09-51-52'
$PYTHON_VIRTUAL_ENV run_scanmatcher.py -i $directory

