#!/usr/bin/env python3
#Testing script for recording functions
import logging
import time
import os
import sys
# run this exaple from the spot_driver directory, using
# python3 scripts/examples/recording_example.py
sys.path.append(os.getcwd() + "/src")
from spot_driver.spot_wrapper import SpotWrapper
from spot_driver.utils.graphNav_wrapper import GraphNav
#from spot_driver.spot_task_wrapper import SpotTaskWrapper

# before executing, we assume our users have made the following environment variables in their os: BOSDYN_CLIENT_USERNAME, BOSDYN_CLIENT_PASSWORD, BOSDYN_CLIENT_IP
# To make an environment variable, do something like export <variable-name> = '<value>', or have a bash script

import numpy as np
class RecordingTester:
    def __init__(self, power_off=False):
        '''
        NOTE: This script test the WorldObjectHandler class without requiring 
        any ros specific code. The wrapper could be used with other pure python 
        code.
        ''' 
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
        
        self.graphNav = GraphNav(self.spot._robot, self.spot._logger)

        self.log.setLevel('DEBUG')
        
        self.log.debug('Powering on...')
        self.spot.getLease(hijack=True)
        self.spot.power_on()

    def obtain_recording_test(self):
        '''
        Function that tests downloading of a map
        Commands the robot to stand up, walk one meter forward, and turn 90 degrees
        Downloads the recording to local machine
        '''
        self.log.debug('Standing...')
        self.spot.ensure_arm_power_and_stand()

        self.log.debug('Attemptiong to clear maps...')
        self.spot._clear_graph()

        self.log.debug('Getting status of the recording...')
        self.graphNav.get_recording_status()

        self.log.debug('Attempting to start recording...')
        self.graphNav.record()

        self.log.debug('Getting recording status')
        self.graphNav.get_recording_status()

        self.log.debug('Walking forward ...')
        self.spot.trajectory_cmd(1, 0, 90, 20)

        self.log.debug('Attempting to stop recording...')
        self.graphNav.stop_recording()

        self.log.debug('Getting recording status')
        self.graphNav.get_recording_status()

        self.log.debug('Attempting to download the recording')
        self.graphNav.download_recording()

    def __del__(self): #Destructor
        time.sleep(5)
        if self.power_off: self.spot.safe_power_off()
        self.spot.releaseLease()
        self.log.debug(f'Done')

if __name__=='__main__':
    Testrun = RecordingTester()
    Testrun.obtain_recording_test()
    del Testrun