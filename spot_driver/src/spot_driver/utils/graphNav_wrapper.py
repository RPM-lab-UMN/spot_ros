import time
import math
import os
import sys
import argparse
import google.protobuf.timestamp_pb2
import logging

from google.protobuf import wrappers_pb2 as wrappers

from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2


import bosdyn.client.channel
import bosdyn.client.util
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient

from bosdyn.client import math_helpers
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat
from bosdyn.client.recording import GraphNavRecordingServiceClient



from bosdyn.client.frame_helpers import (
    get_odom_tform_body,
    ODOM_FRAME_NAME,
    BODY_FRAME_NAME,
    HAND_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    VISION_FRAME_NAME,
    get_se2_a_tform_b,
    get_a_tform_b
)
from bosdyn.client.graph_nav import GraphNavClient

from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.local_grid import LocalGridClient


from bosdyn.client.lease import LeaseKeepAlive
from . import graph_nav_util
import numpy as np
class GraphNav(object):
    def __init__(self, robot, logger, spot_wrapper, spot_task_wrapper, 
                 download_filepath=os.getcwd(), client_metadata=None):
        # Keep the robot instance and it's ID.
        self._robot = robot
        # Force trigger timesync.
        self._robot.time_sync.wait_for_sync()
        # Keep logger for debug, info, and error statements
        self._logger = logger
        # Build the spot wrapper to get access to tasks defined in spot_wrapper
        self._spot_wrapper = spot_wrapper
        # Build the spot task wrapper to get access to tasks defined in spot_task_wrapper
        self._spot_task_wrapper = spot_task_wrapper

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
        self._recording_client = self._robot.ensure_client(
                        GraphNavRecordingServiceClient.default_service_name)
        
        # Set up the local grid client
        self._local_grid_client = self._robot.ensure_client(
                        LocalGridClient.default_service_name)
        

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

    def download_recording(self, path = None):
        if(path is None):
            path = self._download_filepath
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
    


    def list_graph(self):
        """List waypoint ids of garph_nav
        Returns: List of waypoint ids from the graph. These Ids are also printed by the logger
        """
        ids, eds = self._list_graph_waypoint_and_edge_ids()
        # skip waypoint_ for v2.2.1, skip waypoint for < v2.2
        return [
            v
            for k, v in sorted(
                ids.items(), key=lambda id: int(id[0].replace("waypoint_", ""))
            )
        ]
    

    '''
    Function to call the robot to navigate to a target place
    '''
    def navigate_to(
        self,
        navigate_to_target,
        initial_localization_fiducial=True,
        initial_localization_waypoint=None,
    ):
        """navigate with graph nav.

        Args:
           navigate_to : Waypont id string for which waypoint to navigate to
           initial_localization_fiducial : Tells the initializer whether to use fiducials
           initial_localization_waypoint : Waypoint id string of current robot position (not needed if using fiducials)
        """


        assert not self._robot.is_estopped(), "Robot is estopped. cannot complete navigation"
        assert self._spot_wrapper.check_is_powered_on(), "Robot not powered on, cannot complete navigation"
        assert self._spot_wrapper._lease != None, "No lease claim, cannot complete navigations"
        
        if self._graph_nav_client.download_graph() == None:
            self._logger.error("No graph is uploaded to the robot, cannot complete navigation")
            return
        if initial_localization_fiducial:
            self._set_initial_localization_fiducial()
        if initial_localization_waypoint:
            self._set_initial_localization_waypoint([initial_localization_waypoint])

        
        self._list_graph_waypoint_and_edge_ids()
        self._get_localization_state()
        resp = self._navigate_to([navigate_to_target])

        return resp
    

     ###################################################################
    ## copy from spot-sdk/python/examples/graph_nav_command_line/graph_nav_command_line.py
    def _get_localization_state(self):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        self._logger.info("Got localization: \n%s" % str(state.localization))
        odom_tform_body = get_odom_tform_body(
            state.robot_kinematics.transforms_snapshot
        )
        self._logger.info(
            "Got robot state in kinematic odometry frame: \n%s" % str(odom_tform_body)
        )

    def _set_initial_localization_fiducial(self):
        """Trigger localization when near a fiducial."""
        robot_state = self._spot_wrapper._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            ko_tform_body=current_odom_tform_body,
        )

    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            self._logger.error("No waypoint specified to initialize to.")
            return
        # waypoint id can either be passed in as a single string or a list of waypoints,
        # in which case the first will be used.
        if isinstance(args[0], list):
            waypoint_id = args[0][0]
        else:
            waypoint_id = args[0]
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            waypoint_id,
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return
        robot_state = self._spot_wrapper._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance=0.2,
            max_yaw=20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body,
        )

    def _list_graph_waypoint_and_edge_ids(self):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Empty graph.")
            return
        self._current_graph = graph

        localization_id = (
            self._graph_nav_client.get_localization_state().localization.waypoint_id
        )

        # Update and print waypoints and edges
        (
            self._current_annotation_name_to_wp_id,
            self._current_edges,
        ) = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, self._logger
        )
        return self._current_annotation_name_to_wp_id, self._current_edges
    
    def extract_waypoint_and_edge_points(self):
        """
        Extract publishable data for graph waypoints and edges,
        which include the waypoint's pose and id
        Returns: tuple, the first element being a dictionary mapping waypoint ids to poses
        and the second element being a list of tuples, where each contains the waypoint ids
        of the two points being connected
        """
        graph = self._graph_nav_client.download_graph()
        edges = graph.edges
        ids_to_waypoint_poses = {wp.id: bdSE3Pose.from_proto(wp.waypoint_tform_ko).inverse() for wp in graph.waypoints}
        publishable_edges = []
        for edge in edges:
            publishable_edges.append((edge.id.from_waypoint, edge.id.to_waypoint))

        return ids_to_waypoint_poses, publishable_edges

    def extract_point_clouds_from_graph(self):
        """
        Extract point cloud data from the robot's active GraphNav map.
        Returns: a N x 3 numpy array of x,y,z point coordinates of the waypoint's point clouds.
        """
        graph = self._graph_nav_client.download_graph()
        waypoints = graph.waypoints
        waypoint_snapshots = {}
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) > 0:
                try:
                    waypoint_snapshots[waypoint.snapshot_id] = self._graph_nav_client.download_waypoint_snapshot(
                        waypoint.snapshot_id)
                except Exception:
                    # Failure in downloading waypoint snapshot. Continue to next snapshot.
                    self._logger.error("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
        data = None
        for wp in waypoints:
            snapshot = waypoint_snapshots[wp.snapshot_id]
            cloud = snapshot.point_cloud
            odom_tform_cloud = get_a_tform_b(cloud.source.transforms_snapshot, ODOM_FRAME_NAME,
                                            cloud.source.frame_name_sensor)
            
            point_cloud_data = np.frombuffer(cloud.data, dtype=np.float32).reshape(int(cloud.num_points), 3)
            '''
            waypoint_tform_odom = bdSE3Pose.from_proto(wp.waypoint_tform_ko)
            waypoint_tform_cloud = waypoint_tform_odom * odom_tform_cloud
            transformed_points = waypoint_tform_cloud.transform_cloud(point_cloud_data)
            '''
            # A test here: return the point cloud in odom frame
            transformed_points = odom_tform_cloud.transform_cloud(point_cloud_data)
            if data is None:
                data = transformed_points
            else:
                data = np.concatenate((data, transformed_points))
        return data
    

    def _upload_graph_and_snapshots(self, upload_filepath):
        """Upload the graph and snapshots to the robot."""
        self._logger.info("Loading the graph from disk into local storage...")
        with open(upload_filepath + "/graph", "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            self._logger.info(
                "Loaded graph has {} waypoints and {} edges".format(
                    len(self._current_graph.waypoints), len(self._current_graph.edges)
                )
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(
                upload_filepath + "/waypoint_snapshots/{}".format(waypoint.snapshot_id),
                "rb",
            ) as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[
                    waypoint_snapshot.id
                ] = waypoint_snapshot
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            with open(
                upload_filepath + "/edge_snapshots/{}".format(edge.snapshot_id), "rb"
            ) as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        # Upload the graph to the robot.
        self._logger.info("Uploading the graph and snapshots to the robot...")
        self._graph_nav_client.upload_graph(
            lease=self._lease.lease_proto, graph=self._current_graph
        )
        # Upload the snapshots to the robot.
        for waypoint_snapshot in self._current_waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self._logger.info("Uploaded {}".format(waypoint_snapshot.id))
        for edge_snapshot in self._current_edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self._logger.info("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            self._logger.info(
                "Upload complete! The robot is currently not localized to the map; please localize",
                "the robot before attempting a navigation command.",
            )


    def register_nav_interruption_callback(self, callback):
        """register ros callbacks to execute when spot detects an obstacle during navigation"""
        self._nav_interruption_callback = callback


    def _navigate_to(self, *args):
        """Navigate to a specific waypoint. Uses bosdyn client's navigate_to function, while periodically
        detecting obstacles near spot. If an obstacle is detected, the functions in the nav_interrupt_callbacks
        list are called.
        Parameters: *args, waypoint id for where to navigate to, or list of waypoints, the first of which will be used as 
        the destination.
        """
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self._logger.info("No waypoint provided as a destination for navigate to.")
            return
        assert not self._robot.is_estopped(), "Robot is estopped. cannot complete navigation"
        assert self._spot_wrapper.check_is_powered_on(), "Robot not powered on, cannot complete navigation"
        assert self._spot_wrapper._lease != None, "No lease claim, cannot complete navigations"
        self._spot_wrapper._lease = self._spot_wrapper._lease_wallet.get_lease()
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return False, "Destination waypoint not found"
        if not self._spot_wrapper.toggle_power(should_power_on=True):
            self._logger.info(
                "Failed to power on the robot, and cannot complete navigate to request."
            )
            return False, "Failed to power on the robot, and cannot complete navigate to request."

        # Stop the lease keepalive and create a new sublease for graph nav.
        self._spot_wrapper._lease = self._spot_wrapper._lease_wallet.advance()
        sublease = self._spot_wrapper._lease.create_sublease()
        self._spot_wrapper._lease_keepalive.shutdown()

        # Navigate to the destination waypoint.
        is_finished = False
        nav_to_cmd_id = -1
        obstacle_detected_response = self.detect_obstacles_near_spot(0.25)
        num_navigation_calls = 0
        while not is_finished:
            num_navigation_calls += 1
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).\
            self._logger.info(obstacle_detected_response)
            if obstacle_detected_response[0] == True:
                time.sleep(2)
                self._logger.info("Obstacle detected, removing it from path")
                obstacle_feedback = {}
                grid = self.get_obstacle_distance_grid()
                # get spot's current location to return to after dragging the chair
                obstacle_feedback["spot_location_odom"] = self._spot_wrapper._transform_bd_pose(bdSE3Pose(0, 0, 0, bdQuat()), BODY_FRAME_NAME, ODOM_FRAME_NAME)
                # send the location of where to move the obstacle in spot's body frame
                # use obstacle_protocol feedback once it is improved
                # TODO: validate obstacle_protocol and find an appropriate place to put the obstacle
                # obstacle_feedback["obstacle_destination_body"] = self.obstacle_protocol(grid)
                obstacle_feedback["obstacle_destination_odom"] = self._spot_wrapper._transform_bd_pose(bdSE3Pose(0, -1.5, 0, bdQuat()), BODY_FRAME_NAME, ODOM_FRAME_NAME)
                # send the rough location of the obstacle in spot's body frame
                obstacle_feedback["obstacle_location_body"] = obstacle_detected_response[1]
                #self._logger.info(str(obstacle_detected_response[1]))
                self._nav_interruption_callback(obstacle_feedback)
                self._logger.info("Callback made to send obstacle movement command")
            # commands are issued to last for 1 second, and issued on a loop to regularly check for
            # obstacles detected
            nav_to_cmd_id = self._graph_nav_client.navigate_to(
                destination_waypoint, 1.0, leases=[sublease.lease_proto]
            )
            self._logger.info(str(num_navigation_calls) + " calls made to bosdyn navigate_to")
            # TODO: Move this onto a separate thread and check it more frequently
            # adjust the distance threshold for detecting obstacles here.
            obstacle_detected_response = self.detect_obstacles_near_spot(0.25)


            # Sleep 0.5 seconds to allow for command execution.
            # adjust this time if needed to check for obstacles more/less frequently
            time.sleep(0.5) 
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        self._spot_wrapper._lease = self._spot_wrapper._lease_wallet.advance()
        self._spot_wrapper._lease_keepalive = LeaseKeepAlive(self._spot_wrapper._lease_client)
        # Update the lease and power off the robot if appropriate.
        if self._spot_wrapper.check_is_powered_on():
            # Sit the robot down + power off after the navigation command is complete.
            self._spot_wrapper.toggle_power(should_power_on=False)

        status = self._graph_nav_client.navigation_feedback(nav_to_cmd_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            return True, "Successfully completed the navigation commands!"
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            return (
                False,
                "Robot got lost when navigating the route, the robot will now sit down.",
            )
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            return (
                False,
                "Robot got stuck when navigating the route, the robot will now sit down.",
            )
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            return False, "Robot is impaired."
        else:
            return False, "Navigation command is not complete yet."


    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self._graph_nav_client.clear_graph()

    


    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self._logger.error(
                "Robot got lost when navigating the route, the robot will now sit down."
            )
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self._logger.error(
                "Robot got stuck when navigating the route, the robot will now sit down."
            )
            return True
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            self._logger.error("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint2, to_waypoint=waypoint1
                    )
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint1, to_waypoint=waypoint2
                    )
        return None



    def get_obstacle_distance_grid(self):
        """
        Gives the obstacle distance grid of the robot, which represents how close points surrounding the robot are
        to obstacles.

        returns: a 2-dimensional numpy array, with each entry (x, y) having a value representing how far away it is from
        the nearest obstacle. Distances are measured in cells, which are roughly 0.03 meters"""
        # get local grid proto
        obstacle_distance_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0]
        cells_pz = np.frombuffer(obstacle_distance_proto.local_grid.data, np.int16)
        cells_pz_full = []
        # For each value of rle_counts, we expand the cell data at the matching index
        # to have that many repeated, consecutive values.
        for i in range(0, len(obstacle_distance_proto.local_grid.rle_counts)):
            for j in range(0, obstacle_distance_proto.local_grid.rle_counts[i]):
                cells_pz_full.append(cells_pz[i])
        # get dimensions of obstacle_distance_grid, usually 128x128
        x_dim = obstacle_distance_proto.local_grid.extent.num_cells_x
        y_dim = obstacle_distance_proto.local_grid.extent.num_cells_y
        full_cells_array = np.array(cells_pz_full)
        # reshape the distances list to fit the dimensions of the grid
        value_scale = obstacle_distance_proto.local_grid.cell_value_scale
        value_offset = obstacle_distance_proto.local_grid.cell_value_offset
        full_cells_array = np.reshape(full_cells_array, (y_dim, x_dim))
        float_array = full_cells_array.astype(float)
        float_array *= value_scale
        float_array += value_offset
        return float_array

    def _get_transform_to_local_grid(self, frame = BODY_FRAME_NAME):
        """
        Transform an SE3 pose into the obstacle_distance frame
        Snapshots must be taken from the local grid instead of the robot_state client,
        so _transform_bd_pose cannot be used
        Parameters:
            frame: the reference frame of the transformation
        Returns a_tform_b transformation from the provided frame to the local grid
        """
        obstacle_distance_grid_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0]
        # get snapshot from local grid instead of robot state client
        # _transform_bd_pose assumes snapshot comes from robot state,
        # so it cannot be used for this
        grid_snapshot = obstacle_distance_grid_proto.local_grid.transforms_snapshot
        # get name of local grid frame
        grid_frame = obstacle_distance_grid_proto.local_grid.frame_name_local_grid_data
        # get a transformation from the provided frame into the local grid frame
        return  get_a_tform_b(grid_snapshot, grid_frame, frame)

    def _get_obstacle_grid_coordinates(self, pose, transform, obstacle_distance_grid_proto = None):
        """
        Get the coordinates of a pose in spot's obstacle_distance_grid
        Parameters:
            pose, a bdSE3pose to get the coordinates of
            transform: A transformation of the pose into the obstacle_distance grid's frame.
                This can be obtained by calling _get_transform_to_local_grid
            obstacle_distance_grid_proto: proto for obstacle distance grid obtained from the client.
                if None (default), will be extracted from the local grid client.

        """
        if obstacle_distance_grid_proto == None:
            obstacle_distance_grid_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0]
        pose_in_obstacle_grid =  transform * pose
        cell_size = obstacle_distance_grid_proto.local_grid.extent.cell_size
        # translate the position in the grid frame into the coordinates in the grid
        grid_coordinates = [round(pose_in_obstacle_grid.position.y / cell_size), round(pose_in_obstacle_grid.x / cell_size)]
        return grid_coordinates
    


    def check_proximity_to_obstacles(self, poses, frame = BODY_FRAME_NAME):
        """
        Get the distance from the nearest obstacle of a given list of poses, in a given frame, using the
        obstacle_distance grid
        Parameters: poses (pbSE3), a list of positions to check,
                    frame: the frame the pose is in
        Returns: the distance of the location in the pose from the nearest obstacle, in
        meters
        """
        obstacle_distance_grid_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0]
        obstacle_distance_grid = self.get_obstacle_distance_grid()
        distances = []
        for pose in poses:
            T = self._get_transform_to_local_grid()
            grid_coordinates = self._get_obstacle_grid_coordinates(pose, T, obstacle_distance_grid_proto)
            x_dim = obstacle_distance_grid_proto.local_grid.extent.num_cells_x
            y_dim = obstacle_distance_grid_proto.local_grid.extent.num_cells_y
            if (grid_coordinates[0]) >= y_dim or grid_coordinates[0] < 0 or (grid_coordinates[1]) >= x_dim or grid_coordinates[1] < 0:
                self._logger.error("Specified point not within obstacle distance grid")
                distances.append(0)
            # get the obstacle distance at that location in the grid
            else:
                distances.append(obstacle_distance_grid[grid_coordinates[0]][grid_coordinates[1]])
        return distances
    


    def detect_obstacles_near_spot(self, threshold = 0.5):
        """
        Purpose: detect whether points on spot's body are within a certain distance threshold of an obstacle
        Parameters: threshold, the maximum distance you want to allow spot to be from an obstacle.
        Returns: Tuple containing a Boolean, whether any points on spot's body are within the threshold distance of an obstacle,
            and a SE3Pose estimating where the obstacle is, in spot's body frame
        """
        # get a list of points on the edges of spots body
        poses = []
        width = 0.5
        length = 1.1
        # get poses that are on the edges and corners of spot's body
        for x in range(-1, 2):
            for y in range(-1, 2):
                poses.append(bdSE3Pose(x * length / 2 , y * width / 2, 0, bdQuat()))
        distances_list = self.check_proximity_to_obstacles(poses)
        closest_pose = poses[distances_list.index(min(distances_list))]
        if min(distances_list) < threshold:
            # add the distance detected by the obstacle grid to the closest pose
            self._logger.info(str(closest_pose))
            closest_pos_vec3 = math_helpers.Vec3.from_proto(closest_pose)
            vector_offset = closest_pos_vec3 / closest_pos_vec3.length() * min(distances_list)
            self._logger.info(min(distances_list))
            self._logger.info(str(vector_offset))
            closest_pose = bdSE3Pose(closest_pose.x + vector_offset.x, closest_pose.y + vector_offset.y, closest_pose.z + vector_offset.z, bdQuat())
            self._logger.info(str(closest_pose))
            return True, closest_pose
        return False, None



    def obstacle_protocol(self, grid):
        """
        Purpose: Determines an open space to move an obstacle once the obstacle has been detected near spot
        Parameters:
            grid(nxn array): the local grid snapshot that returned the issue
        Returns: a bdSE3Pose in the body frame that is safe to relocate the object
        """
        #Ensure a Stop of all movement to avoid collision
        self.stop()
        self._logger.info("Obstacle ahead, trying to find a safe place to relocate it")
        #Determine a safe location to move the obstacle
        possible_obstacle_destinations = self._find_safe_place_for_obstacle(grid)
        if(len(possible_obstacle_destinations) == 0): #Nothing was found, so spot sits down and waits
            self._logger.error("No good relocation places located. Spot will sit down now")
            self.sit()
            return
        self._logger.info("Successfully generated candidate list of safe places, now trying to find best one")
        #Extract Spot's location within the obstacle grid to determine the closes safe point
        tform_to_obstacle_grid = self._get_transform_to_local_grid()
        spot_location = self._get_obstacle_grid_coordinates(bdSE3Pose(0, 0, 0, bdQuat()), tform_to_obstacle_grid)
        spot_location = np.array(spot_location)
        #Weed out the extraneous solutions
        best_obstacle_destination = self._weed_out_locations(possible_obstacle_destinations, spot_location)
        if(best_obstacle_destination is None): #Nothing was found, so spot sits down and waits
            self._logger.error("No good relocation places located. Spot will sit down now")
            self.sit()
            return
        self._logger.info("Ideal destination located: ") #Debug statement
        #Convert the best obstacle destination back to a body frame coordinate, so spot can navigate there
        tform_to_body_frame = tform_to_obstacle_grid.inverse()
        obstacle_distance_grid_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0]
        cell_size = obstacle_distance_grid_proto.local_grid.extent.cell_size
        best_obstacle_destination_body = tform_to_body_frame * bdSE3Pose(best_obstacle_destination[0]*cell_size, best_obstacle_destination[1]*cell_size, 0, bdQuat())
        best_obstacle_destination_body_coords = bdSE3Pose(best_obstacle_destination_body.x, best_obstacle_destination_body.y, 0, bdQuat())
        best_obstacle_destination_odom = self._transform_bd_pose(best_obstacle_destination_body_coords, BODY_FRAME_NAME, ODOM_FRAME_NAME)
        return best_obstacle_destination_odom

    def _find_safe_place_for_obstacle(self, grid_array, *args):
        """
        Purpose: Helper function to determine all the safe spaces to relocate the object
        Parameters: grid_array. nxn numpy array of integer values that detail how far each coordinate is away from the obstacle
        Returns: Safe_places. List of coordinates in obstacle_grid frame that are determined "safe" to relocate the object
        """
        safe_places = [] #Ideally, there will be many safe place to choose from
        # We will want to store the list of candidates to relocated our chair to
        # Another function will prune the list for the best place
        # Loop through grid, searching candidate points
        rows = len(grid_array)
        columns = len(grid_array[0])
        for x in range(rows):
            for y in range(columns):
                potential_point = grid_array[x][y]
                if(potential_point >= 0.40): #Step 2: confirming the point, but also its neighbors
                    if(self._ensure_neighbors(x,y, grid_array)):
                        safe_places.append((x,y))
        if(len(safe_places) == 0):
            self._logger.error("There are no safe places that could be found within the obstacle grid")
        return safe_places

    def _ensure_neighbors(self, i, j, grid):
        """
        Purpose: Helper function Confirm the immediate surroundings of a candidate safe point are also safe.
        Parameters:
            i: row coordinate of a candidate point
            j: column coordinate of the candidate point
            grid: obstacle grid
        Returns:
            A boolean determining whether all immediate surroundings are safe
        """
        rows = len(grid) #Extract rows
        columns = len(grid[0]) #Extract columns
        # Extract microgrid of maximum 20x20 with i,j at the center, since a cell size is approximately 3 cm
        # First, find the boundaries of the x we can iterate over
        min_x = min(0, i-8)
        max_x = max(i+8,rows-1)
        min_y = min(0,j-8)
        max_y = max(j+8,columns-1)

        # Next, extract the subgrid from the input grid for checking all the values
        microgrid = grid[min_x:max_x][min_y:max_y]
        # Loop over microgrid to check neighboring values
        for x in range(len(microgrid)):
            for y in range(len(microgrid[x])):
                if(microgrid[x][y] < 0.07):
                    return False
        return True

    def _weed_out_locations(self, candidates, spot_position):
        """
        Purpose: Helper function that prunes the list of candidate points to find the best one. The chosen
        location will be between 1 m and 2 m from spot, but these margins can be changed
        Parameters:
            candidates: list of points that have been verified with their immediate surroundings
            Spot_position: location of spot on the obstacle grid
        Returns:
            best_location: obstacle grid coordinates of the best location
        """
        if(len(candidates) ==0):
            self._logger.error("Candidates list is empty, prompting spot to sit down as no way to relocate object exists")
            return None
        new_candidates = []
        obstacle_distance_grid_proto = self._local_grid_client.get_local_grids(["obstacle_distance"])[0] #Have to translate distance from real-life to cell-sizes
        cell_size = obstacle_distance_grid_proto.local_grid.extent.cell_size
        for candidate in candidates:
            # Edit the distance margins in this line
            if(np.linalg.norm(candidate-spot_position) <= 2 /cell_size and np.linalg.norm(candidate-spot_position) > 1/cell_size): # We want it reasonably out of the way
                new_candidates.append(candidate)
        if(len(new_candidates) == 0):
            self._logger.error("All locations are more than 2 meters away or less than half a meter away. This is extraneous in terms of relocation")
            return None
        best_location = new_candidates[0] #Default return value
        best_location = np.array(best_location) #convert to linalg array
        smallest_dist = np.linalg.norm(best_location-spot_position) #Calculate Euclidean distance, use largest because smallest would be on top of spot and still in the way
        for candidate in new_candidates: #Loop through entire array of candidates for the best position
            candidate = np.array(candidate)
            dist = np.linalg.norm(candidate-spot_position)
            if(dist < smallest_dist): #Run a comparison with the currently identified best
                best_location = candidate
                smallest_dist = dist
        return best_location