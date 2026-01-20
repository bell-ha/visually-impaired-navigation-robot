import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jongha/Desktop/GitHub/visually-impaired-navigation-robot/install/blind_nav_system'
