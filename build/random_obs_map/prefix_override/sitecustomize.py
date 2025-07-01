import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yunfeibi/Ros_projects/ros2-randomly-generate-obstacles/install/random_obs_map'
