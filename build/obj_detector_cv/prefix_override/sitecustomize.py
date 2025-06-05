import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fazal/Documents/dev/phnx_ws/src/road_detectors/install/obj_detector_cv'
