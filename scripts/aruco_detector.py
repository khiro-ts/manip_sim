#!/usr/bin/env python

import numpy as np

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco

from move_arm_pos_client import move_arm_pos_client

class ArUcoDetector:
    def __init__(self, id):
        rospy.init_node('aruco_detector', anonymous=True)

        self.search_pos = [2.258788, 1.159874, -1.591275, -1.557773, -1.234436, -0.606271]
        result = move_arm_pos_client(0, self.search_pos)
        rospy.loginfo("arm motion: {}".format(result))
        
        self.bridge = CvBridge()
        self.marker_id_to_detect = id
        self.image_sub = rospy.Subscriber("/fingercam/color/image_raw", Image, self.image_callback)
        rospy.loginfo("searching for ArUco marker id {} ...".format(self.marker_id_to_detect))
        
    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None and self.marker_id_to_detect in ids:
            index = np.where(ids == self.marker_id_to_detect)[0][0]
            corners_of_detected_marker = corners[index][0]
            
            rospy.loginfo("ArUco marker id {}: at {}".format(self.marker_id_to_detect, corners_of_detected_marker))

            cv2.putText(cv_img, "ID:{}".format(self.marker_id_to_detect), tuple(corners_of_detected_marker[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
            cv2.drawContours(cv_img, [corners_of_detected_marker.astype(int)], 0, (0, 200, 0), 2)
            
            cv2.imshow("ArUco Detector", cv_img)
            cv2.waitKey(0)
            
            rospy.signal_shutdown("ArUco marker detected")

            
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ArUco Detector")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    ad = ArUcoDetector(0)
    ad.run()