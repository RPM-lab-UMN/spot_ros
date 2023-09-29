#!/usr/bin/env python3
import rospy
import actionlib
from spot_msgs.msg import FindGraspPointAction, FindGraspPointResult, FindGraspPointFeedback
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import cv2
import threading


class FindGraspPoint(object):
    def __init__(self):
       self._server = actionlib.SimpleActionServer(
            "find_grasp_point",
            FindGraspPointAction,
            execute_cb=self._handle_action,
            auto_start=False
       )
       self.bridge = CvBridge()
       self.g_image_click = None
       self.g_image_display = None
       


    def _handle_action(self, request):
        image_ros = request.selected_frame
        action_type = request.action_type

        image_cv2 = self.bridge.imgmsg_to_cv2(image_ros, "rgb8")
        
        
        
        #Start feedback thread
        self._feedback_thread = threading.Thread(target=self._handle_feedback, args=(action_type))
        self._running = True
        self._feedback_thread.start()
        
        pick_x = -1.0
        pick_y = -1.0
        if(action_type == "manual_force"):
            # If the grasp point is asked through a manual force
            pick_x, pick_y = self._get_pick_vec_manual_force(image_cv2)
            self.g_image_click = None
            self.g_image_display = None
        

        if(pick_x != -1.0 and pick_y != -1.0):
            # If a pixel is picked, return True
            self._server.set_succeeded(
                FindGraspPointResult(
                    success= True,
                    pick_x = pick_x,
                    pick_y = pick_y
                )
            )
        else:
            # If the pixel is not updated, return false
            self._server.set_succeeded(
                FindGraspPointResult(
                    success= False,
                    pick_x = -1,
                    pick_y = -1
                )
            )
        cv2.destroyAllWindows()
        # Stop feedback thread
        self._running = False
        self._feedback_thread.join()
    
    def _handle_feedback(self, action_type):
        while not rospy.is_shutdown() and self._running:
            f = FindGraspPointFeedback("The Server is still running... " + "Type: " + action_type)
            self._server.publish_feedback(f)
            rospy.Rate(self.feedback_rate).sleep()
    '''
    Methods to deal with the issue via a manual selection
    '''
    def cv_mouse_callback(self, event, x, y):
    
        clone = self.g_image_display.copy()
        if event == cv2.EVENT_LBUTTONUP:
            self.g_image_click = (x, y)
        else:
            # Draw some lines on the image.
            #print('mouse', x, y)
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to grasp'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)

    
    def _get_pick_vec_manual_force(self, img):
        # Show the image to the user and wait for them to click on a pixel
        image_title = 'Click to grasp'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback)

        self.g_image_display = img
        cv2.imshow(image_title, self.g_image_display)
        while self.g_image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                return None
        return self.g_image_click[0], self.g_image_click[1]

        