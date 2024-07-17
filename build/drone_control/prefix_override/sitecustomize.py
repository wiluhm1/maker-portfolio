import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/zuriy/Documents/bwsi-uav-good/flying_squirrels_architecture/install/drone_control'
