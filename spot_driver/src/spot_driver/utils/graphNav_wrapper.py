import time
import math
import os
import sys
import argparse
import google.protobuf.timestamp_pb2
import logging

from google.protobuf import wrappers_pb2 as wrappers

import bosdyn.client.channel
import bosdyn.client.util
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.client.exceptions import ResponseError
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
from bosdyn.client.power import PowerClient, power_on, safe_power_off
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

class GraphNav(object):
    def __init__(self, robot, logger, download_filepath=os.getcwd(), client_metadata=None):
        # Keep the robot instance and it's ID.
        self._robot = robot
        # Force trigger timesync.
        self._robot.time_sync.wait_for_sync()
        # Keep logger for debug, info, and error statements
        self._logger = logger

        # Filepath for the location to put the downloaded graph and snapshots. Default is the current working directory
        if download_filepath[-1] == "/":
            self._download_filepath = download_filepath + "downloaded_graph"
        else:
            self._download_filepath = download_filepath + "/downloaded_graph"

        # Setup the recording service client.
        self._recording_client = self._robot.ensure_client(
            GraphNavRecordingServiceClient.default_service_name)

        # Create the recording environment.
        self._recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))

        # Setup the graph nav service client.
        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)

        self._map_processing_client = robot.ensure_client(
            MapProcessingServiceClient.default_service_name)

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()

    def should_we_start_recording(self):
        """Helper function to authenticate whether recording can happen"""
        #Step 1: verify that there is no current graph already on the robot
        graph = self._graph_nav_client.download_graph()
        if graph is not None:
            # So the robot already has a graph on it. Now we have to check if this graph has waypoints
            # if it does have waypoints, we should localize to them
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    # Not localized to anything in the map. The best option is to clear the graph or
                    # attempt to localize to the current map.
                    # Returning false since the GraphNav system is not in the state it should be to
                    # begin recording.
                    self._logger.error("Robot already has a map on it. Either clear it or fix localization issues")
                    return False
        # We get here if there is no preloaded map. This means we are good to go!
        return True

    def record(self, *args):
        """Prompts the Robot to record a map of its own motion."""
        should_start_recording = self.should_we_start_recording() #This is run first to give us the ok to record
        if not should_start_recording: #Failed the ready check, report back
            self._logger.error("The system is not in the proper state to start recording.", \
                   "Try using the graph_nav_command_line to either clear the map or", \
                   "attempt to localize to the map.")
            return False, "Not in proper state to start recording"
        try:
            self._recording_client.start_recording(recording_environment=self._recording_environment) #Attempt the recording procedure
            self._logger.info("Successfully started recording a map.")
        except Exception as err: #Any issue in the start-up process will be redirected here
            self._logger.error("Start recording failed: " + str(err))
            return False, "Failed to start Recording"
        return True, "Started recording"
    
    def stop_recording(self, *args):
        """Prompts the Robot to stop recording"""
        """Stop or pause recording a map."""
        """Returns a fail if its not recording in the first place"""
        first_iter = True #Stores the first iteration
        while True: #Keep running every second
            try: #Attempt the stop
                self._recording_client.stop_recording() #Command to stop
                self._logger.info("Successfully stopped recording a map.") #Success
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                # It is possible that we are not finished recording yet due to
                # background processing. Try again every 1 second.
                if first_iter:
                    self._logger.info("Cleaning up recording...")
                first_iter = False
                time.sleep(1.0)
                continue
            except Exception as err:
                self._logger.error("Stop recording failed: " + str(err))
                return False, "Failed to stop recording"
        return True, "Successfully stopped recording"
    
    def get_recording_status(self, *args):
        """Get the recording service's status."""
        status = self._recording_client.get_record_status()
        if status.is_recording:
            self._logger.info("The recording service is on.")
            return True, "Currently Recording"
        else:
            self._logger.info("The recording service is off.")
            return False, "Not recording"
    
    def _write_bytes(self, filepath, filename, data): 
        """Write data to a file. Used for all downloading procedures"""
        """Helper Function"""
        os.makedirs(filepath, exist_ok=True)
        with open(filepath + filename, 'wb+') as f:
            f.write(data)
            f.close()

    def _write_full_graph(self, graph, download_filepath):
        """Download the graph from robot to the specified, local filepath location."""
        """Helper function"""
        graph_bytes = graph.SerializeToString()
        self._write_bytes(download_filepath, '/graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints, download_filepath):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        """Helper function"""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self._logger.error("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue
            self._write_bytes(download_filepath + '/waypoint_snapshots',
                              '/' + waypoint.snapshot_id, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            self._logger.info("Downloaded {} of the total {} waypoint snapshots.".format(
                num_waypoint_snapshots_downloaded, len(waypoints)))
            
    def _download_and_write_edge_snapshots(self, edges, download_filepath): 
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self._logger.error("Failed to download edge snapshot: " + edge.snapshot_id)
                continue
            self._write_bytes(download_filepath + '/edge_snapshots', '/' + edge.snapshot_id,
                              edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            self._logger.info("Downloaded {} of the total {} edge snapshots.".format(
                num_edge_snapshots_downloaded, num_to_download))

    def download_recording(self, path = self._download_filepath):
        """Downloads the graph that has been recorded and writes it into subdirectory"""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Failed to download the graph.")
            return False, "Failed to download recording"
        self._write_full_graph(graph, path)
        self._logger.info("Graph downloaded with {} waypoints and {} edges".format(
            len(graph.waypoints), len(graph.edges)))
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints, path)
        self._download_and_write_edge_snapshots(graph.edges, path)
        return True, "Succesfully downloaded recording"