import logging, logging.config
import argparse
import pychrono as chrono

""" Common functions and miscellaneous functionalities used throughout the whole project.
"""

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--log', help='set logging mode (default = info)', default='info')
parser.add_argument('-plot', help='enable plotting (default = False)', action='store_true')
parser.add_argument('-vis', help='enable visualization (default = False)', action='store_true')
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

def cutoff(maximum, val):
    """Returns the cutoff of val with respect to maximum."""
    if(abs(val)>abs(maximum)):
        logging.debug("max:"+str(maximum))
        logging.debug("val:"+str(val))
        #print(maximum)
        return sign(val)*maximum
    else:
        return val

def setConstraints(link, maxmin):
    limit = link.GetLimit_Rz()
    limit.SetActive(True)
    limit.SetRotation(True)
    limit.SetMaxElastic(0)
    limit.SetMinElastic(0)
    limit.SetMaxCushion(0)
    limit.SetMinCushion(0)
    limit.SetMax(maxmin[0])
    limit.SetMin(maxmin[1])

