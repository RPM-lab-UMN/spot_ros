import logging
import time 
import sys
import os
import numpy as np
from bosdyn.api import local_grid_pb2
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/local_grid_example.py
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
class LocalGridTester:
    def __init__(self, power_off=False):
        self.power_off = power_off
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.log = logging.getLogger("rosout")
        self.log.debug('Starting code.')
        self.spot = SpotWrapper('admin', 
                                'pvwmr4j08osj', 
                                '192.168.80.3',  #'192.168.80.3','10.0.0.3', 
                                logger=self.log,
                                estop_timeout=9.0,)
        self.log.setLevel('DEBUG')
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()

        

        self.log.debug(str(self.spot._local_grid_client.get_local_grid_types()))

        self.log.info(str(self.spot.get_obstacle_distance_grid()))

        self.log.info(str(self.spot.check_proximity_to_obstacles(bdSE3Pose(1.5, 0, 0, bdQuat()))))
if __name__ == "__main__":
    LocalGridTester()