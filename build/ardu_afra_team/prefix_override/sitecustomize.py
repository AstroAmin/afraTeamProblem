import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amin/ros2_ardu_afra_team_ws/install/ardu_afra_team'
