import logging
import time 
import sys
import os
import numpy as np
from bosdyn.api import local_grid_pb2
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/obstacle_interaction_example.py
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
from spot_driver.utils.graphNav_wrapper import GraphNav

class LocalGridTester:
    def __init__(self, power_off=False):
        ###################################################################
        self.power_off = power_off
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.log = logging.getLogger("rosout")
        self.log.debug('Starting code.')
        self.spot = SpotWrapper(os.getenv('BOSDYN_CLIENT_USERNAME'),
                                os.getenv('BOSDYN_CLIENT_PASSWORD'),
                                os.getenv('BOSDYN_CLIENT_IP'),
                                logger=self.log,
                                estop_timeout=9.0,)
        self.log.setLevel('DEBUG')
        self.graphNav = GraphNav(self.spot._robot, self.spot._logger)
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()
        self.spot.stand()
        ######################################################### Default startup protocol
        #Assumptions:
        #1. The grid info is passed as 128x128 array, which will grow as spot moves around (obstacle frame is loosely based on ODOM frame)
        #2. The grid's cells translate to body frame coordinates (see spot_wrapper.py)
        #3. The path to relocate an object is linear is always open (i.e. there's no objects in the way)
        #4. The values in the obstacle grid are given as floats representing a distance away (i.e. a heuristic rating of safety)
        #5. Spot will have some pre-recorded path on, but there is an obstacle now along the path when there wasn't before

    def get_a_path(self, download_path):
        """
        This function is just an example of recording and downloading a path
        This recorded path will be the path spot will attempt to clear, but then a chair will be placed in the way
        """
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
        self.spot.trajectory_cmd(0, 0, 0.6, 2)
        time.sleep(2)
        self.spot.trajectory_cmd(0.5, 0, 0, 2)
        time.sleep(2)
        self.spot.trajectory_cmd(0, 0, -1.2, 2)
        time.sleep(2)
        self.spot.trajectory_cmd(0.5, 0, 0, 2)
        time.sleep(2)

        self.log.debug('Attempting to stop recording...')
        self.graphNav.stop_recording()

        self.log.debug('Getting recording status')
        self.graphNav.get_recording_status()

        self.log.debug('Attempting to download the recording')
        self.graphNav.download_recording(download_path + "/downloaded_graph")

        self.spot._clear_graph()
        self.log.debug("Uploading graph...") #Upload the graph to return it to the first point, very important it starts in the same spot
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

    def upload_path_with_obstacles(self, upload_path):
        """
        This function uses the uploaded graph that was downloaded by the above function, or any likewise downloaded path
        However, a chair or similar obstacle will be placed 
        """
        self.spot._clear_graph()
        self.log.debug("Uploading graph...")
        self.spot._upload_graph_and_snapshots(upload_path)

        self.spot._get_localization_state()

        self.log.debug("localizing ...")
        
        waypoints = self.spot.list_graph()
        # spot can localize to a specific waypoint in the graph
        # in this case, we use the last waypoint spot recorded, as it will be closest to spot's
        # current location
        self.spot._set_initial_localization_waypoint(waypoints[0])

        # next we can tell spot to navigate the graph back to its first recorded waypoint
        self.log.debug("Navigating waypoints...")

        # when using navigate_to, must specify
        # destination waypoint, and localization method.
        # if using waypoints, must specify localization waypoint

        self.spot.navigate_to(waypoints[-1], False, waypoints[0])
        #Hypothetically, this command will be interrupted as the chair will be in the way
        
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')
    
    def use_tablet_for_mapping(self, download_path):
        """
        This function will give the lease to the tablet
        """
        self.graphNav.record()
        self.log.debug("Started recording")
        self.spot.releaseLease()
        self.log.debug("Releasing the lease, use the tablet now for movement...")
        #Take lease with the tablet here and move the robot around for a more insteresting path
        time.sleep(90) #90 second buffer to do stuff
        self.spot.getLease(hijack=True)
        self.log.debug("lease acquired again, stopping the recording and saving the results to the filepath")
        self.graphNav.stop_recording()
        self.graphNav.download_recording(download_path)

if __name__ == "__main__":
    download_path = os.getcwd() + "/scripts/examples"
    upload_path = download_path + "/downloaded_graph"
    testrun = LocalGridTester()
    #testrun.get_a_path(download_path) # run first if there is no predefined path on your OS
    #testrun.use_tablet_for_mapping(download_path) # run if you wish to use the tablet for recording a path
    testrun.upload_path_with_obstacles(upload_path) #run if you wish to test the path uploaded, with and without any obstacles
