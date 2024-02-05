#!/usr/bin/env python

import numpy as np

import rospy
import transformations as tf_trans
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco

from move_arm_pos_client import move_arm_pos_client

class ArUcoDetector:
    def __init__(self, id):
        rospy.init_node('aruco_detector', anonymous=True)

        self.search_pos = [2.2131014, 1.0824423, -1.4139534, -1.7161070, -1.4348431, -0.6246903]
        result = move_arm_pos_client(0, self.search_pos)
        rospy.loginfo("arm motion: {}".format(result))
        
        self.bridge = CvBridge()
        self.marker_id_to_detect = id
        self.image_sub = rospy.Subscriber("/fingercam/color/image_raw", Image, self.image_callback)
        self.camera_mtx = []
        self.dist_coeff = []
        self.frame_id = ""
        self.caminfo_sub = rospy.Subscriber("/fingercam/color/camera_info", CameraInfo, self.caminfo_callback)
        self.tf_broad = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("searching for ArUco marker id {} ...".format(self.marker_id_to_detect))

    def caminfo_callback(self, msg):
        self.camera_mtx = np.array(msg.K, dtype=np.float32).reshape((3,3))
        #self.dist_coeff = np.array(msg.D, dtype=np.float64)
        self.dist_coeff = np.zeros((4,1))
        self.frame_id = msg.header.frame_id
        rospy.loginfo("camera_mtx {}".format(self.camera_mtx))
        rospy.loginfo("camera_dist_coeff {}".format(self.dist_coeff))

        self.caminfo_sub.unregister()
        
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

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], markerLength=0.15, cameraMatrix=self.camera_mtx, distCoeffs=self.dist_coeff)

            quat = quaternion_from_euler(rvec[0][0][0], rvec[0][0][1], rvec[0][0][2])
            trans = tvec[0][0]
            
            rospy.loginfo("ArUco marker id {}: at {}, {} in frame {}".format(self.marker_id_to_detect, rvec[0][0], tvec[0][0], self.frame_id))
            cv2.drawFrameAxes(cv_img, self.camera_mtx, self.dist_coeff, rvec, tvec, 0.15/2)
            cv2.putText(cv_img, "ID:{}".format(self.marker_id_to_detect), tuple(corners_of_detected_marker[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
            cv2.drawContours(cv_img, [corners_of_detected_marker.astype(int)], 0, (0, 200, 0), 2)

            cv2.imshow("ArUco Detector", cv_img)
            cv2.waitKey(0)
            
            tm = TransformStamped()
            tm.header.stamp = rospy.Time.now()
            tm.header.frame_id = self.frame_id
            tm.child_frame_id = 'aruco_' + str(self.marker_id_to_detect)
            tm.transform.translation.x = trans[0]
            tm.transform.translation.y = trans[1]
            tm.transform.translation.z = trans[2]
            tm.transform.rotation.x = quat[0]
            tm.transform.rotation.y = quat[1]
            tm.transform.rotation.z = quat[2]
            tm.transform.rotation.w = quat[3]
            
            self.tf_broad.sendTransform(tm)

            tfg = self.tf_buffer.lookup_transform(tm.child_frame_id, "Link6", rospy.Time(), rospy.Duration(3.0))
            tfc = self.tf_buffer.lookup_transform("ee_link", "base_link", rospy.Time(), rospy.Duration(3.0))
            print(tfg)
            print(tfc)

            #offset_wrt_aruco = [tfg.transform.translation.x/2, 
            #                    tfg.transform.translation.y/2,
            #                    -tfg.transform.translation.z/2,
            #                    0.*tfg.transform.rotation.x,
            #                    0.*tfg.transform.rotation.y,
            #                    0.*tfg.transform.rotation.z,
            #                    0.*tfg.transform.rotation.w
            #                ]
                               

            offset_wrt_aruco = [trans[0], trans[1], -trans[2], 0.,0.,0.,0.] #quat[0], quat[1], quat[2], quat[3]]
            result = move_arm_pos_client(1, offset_wrt_aruco)
            rospy.loginfo("arm motion: {}".format(result))

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
