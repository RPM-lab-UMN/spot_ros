import logging
import time 
import sys
import os
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/mapping_example.py

# IMPORTANT: This example will make spot walk forwards and backwards in a 
# zig-zag pattern. Make sure there is at least 2 meters of space in front of spot
# before executing, we assume our users have made the following environment variables in their os: USER, PASSWD, IP
# To make an environment variable, do something like export <variable-name> = '<value>', or have a bash script
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
from spot_driver.utils.graphNav_wrapper import GraphNav
class MappingWrapperTester:
    def __init__(self, download_path, power_off=False):
        self.power_off = power_off
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.log = logging.getLogger("rosout")
        self.log.debug('Starting code.')
        self.spot = SpotWrapper(os.getenv('USER'), #'admin'
                                os.getenv('PASSWD'), #'pvwmr4j08osj'
                                os.getenv('IP'),  #'192.168.80.3','10.0.0.3', 
                                logger=self.log,
                                estop_timeout=9.0,)
        
        self.graphNav = GraphNav(self.spot._robot, self.spot._logger)
        
        self.log.setLevel('DEBUG')
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()
        
        # example of recording a graph
        self.log.debug('Attemptiong to clear maps...')
        self.spot._clear_graph()

        self.log.debug('Getting status of the recording...')
        self.graphNav.get_recording_status()

        self.log.debug('Attempting to start recording...')
        self.graphNav.record()

        self.log.debug('Getting recording status')
        self.graphNav.get_recording_status()

        # Spot will walk forward in a zig-zag pattern while recording a GraphNav map
        self.log.debug('Walking forward ...')
        self.spot.trajectory_cmd(1, 0, 0, 2)

        time.sleep(2)
        self.spot.trajectory_cmd(0, 0, 0.6, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0.5, 0, 0, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0, 0, -1.2, 3)
        time.sleep(3)
        self.spot.trajectory_cmd(0.5, 0, 0, 3)
        time.sleep(3)

        self.log.debug('Attempting to stop recording...')
        self.graphNav.stop_recording()

        self.log.debug('Getting recording status')
        self.graphNav.get_recording_status()

        self.log.debug('Attempting to download the recording')
        self.graphNav.download_recording()
       
        # example of downloading and navigating a graph
        self.spot._clear_graph()
        self.log.debug("Uploading graph...")
        self.spot._upload_graph_and_snapshots(download_path + "/downloaded_graph")

        self.spot._get_localization_state()

        self.log.debug("localizing ...")
        
        waypoints = self.spot.list_graph()
        # spot can localize to a specific waypoint in the graph
        # in this case, we use the last waypoint spot recorded, as it will be closest to spot's
        # current location
        self.spot._set_initial_localization_waypoint(waypoints[-1])

        # next we can tell spot to navigate the graph back to its first recorded waypoint
        self.log.debug("Navigating waypoints...")

        # when using navigate_to, must specify
        # destination waypoint, and localization method.
        # if using waypoints, must specify localization waypoint

        self.spot.navigate_to(waypoints[0], False, waypoints[-1])
        
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')
if __name__ == "__main__":
    # specify the path to download and upload the graph here
    download_path = os.getcwd()
    MappingWrapperTester(download_path)