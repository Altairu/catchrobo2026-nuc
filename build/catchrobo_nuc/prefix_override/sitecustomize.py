import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/altair/catchrobo2026-nuc/install/catchrobo_nuc'
