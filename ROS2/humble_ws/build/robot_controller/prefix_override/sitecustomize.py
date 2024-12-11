import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/g83r/Desktop/uga/hero-lab/heroswarmv2/ROS2/humble_ws/install/robot_controller'
