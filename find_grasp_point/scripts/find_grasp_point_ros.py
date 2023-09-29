


import rospy
from find_grasp_point_utils import FindGraspPoint



def main(args):
        grap_point_searcher = FindGraspPoint()
        rospy.init_node('find_grasp_point', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

if __name__ == '__main__':
    main()