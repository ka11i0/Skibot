import logging, logging.config
import argparse
import time
import pickle
from os.path import exists

""" Common functions and miscellaneous functionalities used throughout the whole project.
"""

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--log', help='set logging mode (default = info)', default='info')
parser.add_argument('-plot', help='enable plotting (default = False)', action='store_true')
parser.add_argument('-vis', help='visualization via pygame (default = False)', action='store_true')
parser.add_argument('-run', help='run motors (default = False)', action='store_true')
parser.add_argument('-map', help='create motor mapping (default = False)', action='store_true')
parser.add_argument('-test', help='run test on path (default = False)', action='store_true')
parser.add_argument('-save', help='save generated path into csv (default = False)', action='store_true')
args = parser.parse_args()

# Setup logger
logging.config.fileConfig('logging.conf', disable_existing_loggers=False)
logger = logging.getLogger(args.log)


def sign(x):
    """Returns the sign of variable x."""
    if(abs(x)==x):
        return 1
    else:
        return -1

def sat(sat_lim, val):
    """Returns the cutoff of val with respect to maximum."""
    if(abs(val)>abs(sat_lim)):
        logger.debug("max:"+str(sat_lim))
        logger.debug("val:"+str(val))
        return sign(val)*abs(sat_lim)
    else:
        return val

def ardmap(x, io):
    """Taken from arduino map implementation"""
    # io = (in_min, in_max, out_min, out_max)
    # return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return (x - io[0]) * (io[3] - io[2]) / (io[1] - io[0]) + io[2]



