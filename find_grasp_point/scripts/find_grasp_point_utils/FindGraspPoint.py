#!/usr/bin/env python3
import rospy
import os
from spot_msgs.msg import FindGraspPointAction, FindGraspPointResult, FindGraspPointFeedback
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import cv2
import threading
import torch
import torchvision
from DINO.collect_dino_features import *
from DINO.dino_wrapper import *

import matplotlib

class FindGraspPoint(object):
    def __init__(self):
       self._server = actionlib.SimpleActionServer(
            "spot/find_grasp_point",
            FindGraspPointAction,
            execute_cb=self._handle_action,
            auto_start=False
       )
       self.feedback_rate = 5
       self.bridge = CvBridge()
       self.g_image_click = None
       self.g_image_display = None

       self.home_addr = os.path.expanduser('~') + "/repo/robotdev/spot/ros_ws/src/spot_ros/find_grasp_point/scripts/"
       self.DINO_addr = self.home_addr + "DINO"


    def _handle_action(self, request):
        image_ros = request.selected_frame
        action_type = request.action_type

        image_cv2 = self.bridge.imgmsg_to_cv2(image_ros, "rgb8")
        
        
        
        #Start feedback thread
        self._feedback_thread = threading.Thread(target=self._handle_feedback, args=action_type)
        self._running = True
        self._feedback_thread.start()
        
        pick_x = -1.0
        pick_y = -1.0
        if(action_type == "manual_force"):
            # If the grasp point is asked through a manual force
            pick_x, pick_y = self._get_pick_vec_manual_force(image_cv2)
            
        elif (action_type == "DINO"):
            # If the grasp point is asked through DINO feature extractor
            pick_x, pick_y = self._get_pick_vec_DINO(image_cv2)

        self.g_image_click = None
        self.g_image_display = None

        # Return the result
        if(pick_x != -1.0 and pick_y != -1.0):
            # If a pixel is picked, return True
            self._server.set_succeeded(
                FindGraspPointResult(
                    success= True,
                    pick_x = pick_x,
                    pick_y = pick_y
                )
            )
        elif (pick_x == 0.0 and pick_y == 0.0):
            # If the system tries fails to find a good pixel value
            self._server.set_succeded(
                FindGraspPointResult(
                    success= True,
                    pick_x = 0.0,
                    pick_y = 0.0
                )
            )
        else:
            # If an exception occurred, no good pixel value is picked
            self._server.set_aborted(
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
    
    def _handle_feedback(self, action_type, *args):
        while not rospy.is_shutdown() and self._running:
            f = FindGraspPointFeedback("The Server is still running... " + "Type: " + action_type)
            self._server.publish_feedback(f)
            rospy.Rate(self.feedback_rate).sleep()
    '''
    Methods to deal with the issue via a manual selection
    '''
    def cv_mouse_callback(self, event, x, y, flags, param):
    
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
                # The user decides not to pick anything in the frame
                return 0.0, 0.0
        return self.g_image_click[0], self.g_image_click[1]
    
    def _get_pick_vec_DINO(self, img):
        # Hard-code the default cfg for building DINO model
        cfg = {}
        cfg['dino_strides'] = 4
        cfg['desired_height'] = img.shape[0]
        cfg['desired_width'] = img.shape[1]
        cfg['use_16bit'] = False
        cfg['use_traced_model'] = False
        cfg['cpu'] = False
        cfg['similarity_thresh'] = 0.95

        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        torch.backends.cudnn.benchmark = True
        torch.hub.set_dir(self.DINO_addr + "/hub")
       

        # Part II: get the place with high similarity
        model = get_dino_pixel_wise_features_model(cfg = cfg, device = device)


        img_feat = preprocess_frame(img, cfg=cfg)
        img_feat = model(img_feat)
        
        img_feat_norm = torch.nn.functional.normalize(img_feat, dim=1)


        img_feat_eval = torch.load(self.DINO_addr + "/queries/feat1.pt")
        img_feat_eval = img_feat_eval[0].view(1,-1)
        img_feat_eval = img_feat_eval.cuda()


        cosine_similarity = torch.nn.CosineSimilarity(dim=1)  # (1, 512, H // 2, W // 2)

        similarity = cosine_similarity(
            img_feat_norm, img_feat_eval.view(1, -1, 1, 1)
        )
        # Viz thresholded "relative" attention scores
        similarity = (similarity + 1.0) / 2.0  # scale from [-1, 1] to [0, 1]
        
        # A strange bug here... the similarity is flipped!!
        # similarity = 1 - similarity
        # similarity = similarity.clamp(0., 1.)
        similarity_rel = (similarity - similarity.min()) / (
            similarity.max() - similarity.min() + 1e-12
        )
        similarity_rel = similarity_rel[0]  # 1, H // 2, W // 2 -> # H // 2, W // 2
        #similarity_rel[similarity_rel < cfg['similarity_thresh'] ]= 0.0

        similarity_rel = similarity_rel.detach().cpu().numpy()

        similarity_max = np.max(similarity_rel)
        similarity_argmax = np.argmax(similarity_rel)
        
        similarity_filtered = np.zeros(similarity_rel.shape)
        potential_points = []
        for i in range(similarity_rel.shape[0]):
            for j in range(similarity_rel.shape[1]):
                if similarity_rel[i, j] > cfg['similarity_thresh']:
                    potential_points.append([i , j])
                    similarity_filtered[i, j] = 1
        potential_points = np.array(potential_points)


        _, singulars, _ = np.linalg.svd(similarity_filtered)
        singular_ratio = singulars[0] / singulars[1]
        singular_range = (singular_ratio <= 2.3) or (singular_ratio >= 7)

        x_std = np.std(potential_points[:, 1])
        y_std = np.std(potential_points[:, 0])
        std_ratio = x_std / y_std
        std_range = (std_ratio < 4) or (std_ratio > 15.8)

        avg_pick_point = np.mean(potential_points, axis=0)
        if potential_points.shape[0] <= 40 or singular_range or std_range:
            pick_x = 0
            pick_y = 0
        else:
            avg_pick_point = np.mean(potential_points, axis=0)
            pick_x = int(avg_pick_point[1])
            pick_y = int(avg_pick_point[0])
        

        # Find the point with the highest similarity
        # if (similarity_max < cfg['similarity_thresh']):
        #     # If the closest point in the current frame is still
        #     # different from the query point significantly in latent space
        #     pick_x = 0
        #     pick_y = 0
        # else:
        #     pick_y = int(similarity_argmax/(similarity_rel.shape[1]))
        #     pick_x = int(similarity_argmax%(similarity_rel.shape[1]))

        rospy.loginfo("The picked pixel is: ")
        rospy.loginfo([pick_x, pick_y])
        cmap = matplotlib.cm.get_cmap("jet")
        similarity_colormap = cmap(similarity_rel)[..., :3]

        img_to_viz = img #cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # img is in RGB format
        _overlay = img_to_viz.astype(np.float32) / 255
        _overlay = 0.5 * _overlay + 0.5 * similarity_colormap
        
        cv2.circle(_overlay, (pick_x, pick_y), 5, (0, 0, 255), -1) 
        cv2.imshow("Debug image: " + str(round(std_ratio, 2)) + " " + str(round(singular_ratio, 2)), _overlay)
        cv2.waitKey(2000)


        return pick_x, pick_y
        