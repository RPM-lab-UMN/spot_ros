import logging
import time 
import sys
import os
import numpy as np
from bosdyn.api import local_grid_pb2
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat
import matplotlib.pyplot as plt
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
        xs = []
        ys = []
        # Use local grid to find all points in robot's body frame that have a negative obstacle distance
        # meaning that those points are inside of obstacles
        for y_distance in range(-10, 30):
            for x_distance in range(-10, 30):
                distance = self.spot.check_proximity_to_obstacles(bdSE3Pose(x_distance * 0.1, y_distance * 0.1, 0, bdQuat()))
                if distance < 0:
                    self.log.info(str(x_distance * 0.1)+ "," + str(y_distance * 0.1))
                    xs.append(x_distance * 0.1)
                    ys.append(- y_distance * 0.1)
        # Create a figure and axis
        fig, ax = plt.subplots()

        # Plot the points
        ax.plot(ys, xs, 'ro')  # 'ro' specifies red circles as markers

        # Add labels and title
        ax.set_ylabel('Spot Body frame X-axis')
        ax.set_xlabel('Spot Body frame Y-axis')
        ax.set_title('2-D Points Containing Obstacles')

        # Show the plot
        plt.show()
                    
if __name__ == "__main__":
    LocalGridTester()