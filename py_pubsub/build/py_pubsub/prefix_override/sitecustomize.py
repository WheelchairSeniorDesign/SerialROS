import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aec117-fkmj9r3/ros2_ws/src/py_pubsub/install/py_pubsub'
