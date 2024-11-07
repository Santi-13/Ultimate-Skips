import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sanmaster/ultimate_skips/Ultimate-Skips/install/explorer_skips'
