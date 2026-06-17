import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/github/Omni-Directional-Mobile-Robot/install/robotpadi_controller'
