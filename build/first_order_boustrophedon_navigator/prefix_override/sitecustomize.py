import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chakrireddy1974/RAS-SES-598-Space-Robotics-and-AI/install/first_order_boustrophedon_navigator'
