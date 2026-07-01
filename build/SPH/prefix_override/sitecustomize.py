import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/azivinicius/ros2_ws/src/SPH/install/SPH'
