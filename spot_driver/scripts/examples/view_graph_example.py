import logging
import time 
import sys
import os
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/view_graph_example.py
# before executing, we assume our users have made the following environment variables in their os: BOSDYN_CLIENT_USERNAME, BOSDYN_CLIENT_PASSWORD, BOSDYN_CLIENT_IP
# To make an environment variable, do something like export <variable-name> = '<value>', or have a bash script
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
from spot_driver.utils.graphNav_wrapper import GraphNav
class ViewGraphExample:
    def __init__(self, power_off=False):
        self.power_off = power_off
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.log = logging.getLogger("rosout")
        self.log.debug('Starting code.')
        self.spot = SpotWrapper(os.getenv('BOSDYN_CLIENT_USERNAME'), 
                                os.getenv('BOSDYN_CLIENT_PASSWORD'), 
                                os.getenv('SPOT_IP'), 
                                logger=self.log,
                                estop_timeout=9.0,)
        
        self.log.setLevel('DEBUG')
        
        self.graphNav = GraphNav(self.spot._robot, self.spot._logger)
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()
        self.spot._clear_graph()
        self.log.debug('Attempting to start recording...')
        self.graphNav.record()

        self.spot.stand() 
        """ time.sleep(1)
        self.spot.trajectory_cmd(-1, 0, 0, 2)
        time.sleep(2)
        self.spot.trajectory_cmd(0, 0.5, 0, 2)
        time.sleep(2)
        self.spot.trajectory_cmd(1, 0, 0, 2)
        time.sleep(2)
        self.spot.stand()
        time.sleep(2) """

        self.log.debug('Attempting to stop recording...')
        self.graphNav.stop_recording()
        data = self.spot.extract_waypoint_and_edge_points()
        write_ply(data, os.getcwd() + "/scripts/examples/map_point_clouds.ply")
        self.spot._clear_graph()
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')
def write_ply(data, output):
    """
    Writes an ASCII PLY file to the output file path.
    """
    print('Saving to {}'.format(output))
    with open(output, 'w') as f:
        num_points = data.shape[0]
        f.write(
            'ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
            .format(num_points))

        for i in range(0, num_points):
            (x, y, z) = data[i, :]
            f.write('{} {} {}\n'.format(x, y, z))
if __name__ == "__main__":
    ViewGraphExample()