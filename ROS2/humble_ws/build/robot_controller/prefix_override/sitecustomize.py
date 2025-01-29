import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rishab/heroswarmv2/ROS2/humble_ws/install/robot_controller'
