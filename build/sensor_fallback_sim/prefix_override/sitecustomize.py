import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/taewook/sensor_fallback_sim/install/sensor_fallback_sim'
