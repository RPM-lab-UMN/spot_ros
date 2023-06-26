import logging
import time 
import sys
import os
import numpy as np
from bosdyn.api import local_grid_pb2
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/local_grid_example.py
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
class LocalGridTester:
    def __init__(self, power_off=False):
        ###################################################################
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
        ######################################################### Default startup stuff

        self.spot.trajectory_cmd(1, 0, 0, 10) #We will assume there is an obstacle dead ahead, along this path
        time.sleep(3)
        #Now we will test the obstacle protocol to see if Spot can figure out where to move the chair
        list_of_grids = self.spot._local_grid_client.get_local_grid_types()
        grid = self.spot.get_obstacle_distance_grid()
        self.log.debug("Initiating obstacle relocation protocol")
        self.spot.obstacle_protocol(grid)



if __name__ == "__main__":
    LocalGridTester()
