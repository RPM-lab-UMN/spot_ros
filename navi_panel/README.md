# Using Navi-Panel GUI to Issue Graph Navigation Commands
This package adds an interactive panel to RVIZ that can be used to record,
upload, save, and navigate GraphNav maps. These maps allow spot to 
recall previous walks it has made and navigate between places visited previously.
## Recording A Graph
In RVIZ, you can start recording a map by clicking the "Start Recording" Button on the navigation panel
on the left of the screen. The window will then display how many seconds have elapsed sincee the recording began.
Waypoints recorded by spot will appear in RVIZ as purple cylinders at the point spot was when they were recorded,
labeled with their waypoint IDs. Green arrows drawn between these cylinders show the edges of the graph.
If there is already a graph on Spot, that graph will be extended when you start recording. 
If you want to restart with a new graph, clear the previous graph first and then start recording.
## Stopping A Recording
-While recording a graph, the button to start recording toggles to a stop recording button. 
-You must stop recording a graph before you can save, clear, or navigate a graph. 
-When you stop recording, the time elapsed in the window should reset to zero seconds.
## Clearing A Graph
-Spot's graph can be cleared by clicking the "Clear Graph" button in the panel.
-All waypoint and edge markers will be removed from view in RVIZ, and spot will no longer be able to recall the
waypoints that have been recorded.
-Be sure to save a graph before clearing it if you want to use it again.
## Saving A Graph
-GraphNav maps can be saved and used again by spot if it is in the same space where the graph was recorded.
-You must stop recording before you can save a graph
-Clicking the "Save Graph" button will open a dialogue to select a folder to save the graph in.
-Once you select a folder, the graph will be written to a new folder inside the selected folder with the default title of 
"downloaded_graph"
-This folder contains a subfolder with files for each waypoint snapshot, and another folder containing each edge snapshot from the graph.
## Uploading a Graph
-Previously Saved Graphs can be uploaded to spot, and used for navigation in future sessions.
-Clicking the "Upload Graph" button will open a dialogue to select the folder where the graph was saved.
-Once the folder is selected, the waypoints and edges of the graph will be uploaded to spot and displayed in RVIZ.
-If there was already a graph on spot, it will be deleted and replaced by the uploaded graph.
-If objects that were near spot when the graph was recorded are moved significantly, spot
may have difficulty localizing to waypoints during navigation. 
## Navigating a Graph
-Given a localization waypoint and a navigation destination, spot can automatically walk to a destination waypoint.
-In order to issue navigation commands, you cannot be actively recording a graph, and you must have a body lease for spot. You must also power on spot's motors before navigation.
-Clicking on a waypoint marker in RVIZ will prompt the user to use that waypoint as the Localization waypoint
or to navigate to the waypoint.
-You must select a navigation waypoint before selecting a destination. This waypoint should be near spot's current location.
-Selecting a waypoint to "Navigate To" will immediately start the navigation command.
-Spot can get lost when trying to navigate through many waypoint. If this happens, break the navigation up into multiple steps,
going to one waypoint, and then localizing to it when navigating to the next waypoint.
