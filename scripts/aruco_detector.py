#!/usr/bin/env python3

import numpy as np

import rospy
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco

from tf_util import *
from move_arm_pos_client import move_arm_pos_client

class ArUcoDetector:
    def __init__(self, id):
        rospy.init_node('aruco_detector', anonymous=True)

        self.bridge = CvBridge()
        self.marker_id_to_detect = id
        self.marker_size = 0.15
        self.is_detect = False
        self.camera_mtx = []
        self.dist_coeff = []
        self.caminfo_sub = rospy.Subscriber("/fingercam/color/camera_info", CameraInfo, self.caminfo_callback)
        self.image_sub = rospy.Subscriber("/fingercam/color/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/fingercam/color/image_raw/detection", Image, queue_size=1)

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
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None and self.marker_id_to_detect in ids:
            index = np.where(ids == self.marker_id_to_detect)[0][0]
            corners_of_detected_marker = corners[index][0]

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], markerLength=self.marker_size, cameraMatrix=self.camera_mtx, distCoeffs=self.dist_coeff)

            rospy.loginfo("ArUco marker id {}: at {}, {} in frame {}".format(self.marker_id_to_detect, rvec[0][0], tvec[0][0], self.frame_id))
            cv2.drawFrameAxes(cv_img, self.camera_mtx, self.dist_coeff, rvec, tvec, self.marker_size/2)
            cv2.putText(cv_img, "ID:{}".format(self.marker_id_to_detect), tuple(corners_of_detected_marker[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
            cv2.drawContours(cv_img, [corners_of_detected_marker.astype(int)], 0, (0, 200, 0), 2)
            #cv2.imshow("ArUco Detector", cv_img)
            #cv2.waitKey(0)
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.image_pub.publish(img_msg)

            self.is_detect = True

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ArUco Detector")
            cv2.destroyAllWindows()

            
class SampleDetector(ArUcoDetector):
    def __init__(self):

        rospy.init_node("aruco_detector", anonymous=True)

        # move arm to search pose
        self.search_pose = [2.2131014, 1.0824423, -1.4139534, -1.7161070, -1.4348431, -0.6246903]
        rospy.loginfo("move arm to searching pose")
        result = move_arm_pos_client(0, self.search_pose)

        rospy.sleep(1.0) # wait to finish arm motion 
        
        # detection node
        super().__init__(id=0)
        self.frame_id = ""
        self.tf_broad = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_pose = []
        self.approach_offset = 0.2


    def image_callback(self, msg):

        if self.is_detect:
            return

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

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], markerLength=self.marker_size, cameraMatrix=self.camera_mtx, distCoeffs=self.dist_coeff)

            rospy.loginfo("ArUco marker id {}: at {}, {} in frame {}".format(self.marker_id_to_detect, rvec[0][0], tvec[0][0], self.frame_id))
            cv2.drawFrameAxes(cv_img, self.camera_mtx, self.dist_coeff, rvec, tvec, self.marker_size/2)
            cv2.putText(cv_img, "ID:{}".format(self.marker_id_to_detect), tuple(corners_of_detected_marker[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
            cv2.drawContours(cv_img, [corners_of_detected_marker.astype(int)], 0, (0, 200, 0), 2)
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            self.image_pub.publish(img_msg)

            self.target_pose = self.get_target_pos(rvec[0][0], tvec[0][0])
            self.is_detect = True

    def get_target_pos(self, rvec, tvec):
        
        quat = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        
        # tf from camera to detected marker
        tm = TransformStamped()
        tm.header.stamp = rospy.Time.now()
        tm.header.frame_id = self.frame_id
        tm.child_frame_id = 'aruco_' + str(self.marker_id_to_detect)
        tm.transform.translation.x = tvec[0]
        tm.transform.translation.y = tvec[1]
        tm.transform.translation.z = tvec[2]
        tm.transform.rotation.x = quat[0]
        tm.transform.rotation.y = quat[1]
        tm.transform.rotation.z = quat[2]
        tm.transform.rotation.w = quat[3]

        self.tf_broad.sendTransform(tm)

        # tf from arm_base to marker
        tbm = self.tf_buffer.lookup_transform("arm_base_link", tm.child_frame_id, rospy.Time(), rospy.Duration(5.0))

        #goal_quat = quaternion_from_euler(0, 0, 0, axes='sxyz')  # for debug
        #grip_quat = quaternion_multiply(quat, goal_quat)
            
        # tf from moveit base to approach position (debug)
        tg = TransformStamped()
        tg.header.stamp = rospy.Time.now()
        tg.header.frame_id = 'arm_base_link'
        tg.child_frame_id = 'grip_' + str(self.marker_id_to_detect)
        tg.transform.translation.x = tbm.transform.translation.x
        tg.transform.translation.y = tbm.transform.translation.y
        tg.transform.translation.z = tbm.transform.translation.z + self.approach_offset 
        tg.transform.rotation.x = 1.0  #goal_quat[0]  
        tg.transform.rotation.y = 0.0  #goal_quat[1]  
        tg.transform.rotation.z = 0.0  #goal_quat[2] 
        tg.transform.rotation.w = 0.0  #goal_quat[3] 

        self.tf_broad.sendTransform(tg)
    
        target_pose = [tbm.transform.translation.x, 
                            tbm.transform.translation.y,
                            tbm.transform.translation.z + self.approach_offset,  
                            1.0,  #goal_quat[0],
                            0.0,  #goal_quat[1], 
                            0.0,  #goal_quat[2],
                            0.0,  #goal_quat[3]
                        ]

        return target_pose

    def wait_for_target(self, timeout=5):
        
        duration = rospy.Duration(timeout)

        rospy.loginfo("search id {} for {} sec".format(self.marker_id_to_detect, timeout))
        start = rospy.Time.now()
        try:
            while not self.is_detect and not rospy.is_shutdown():
                if rospy.Time.now() - start > duration:
                    rospy.logwarn("search timeout")
                    return False
                rospy.sleep(0.1)

            rospy.loginfo("move arm to detected target: {}".format(self.target_pose))
            result = move_arm_pos_client(1, self.target_pose)
            rospy.loginfo("arm motion: {}".format(result))

            rospy.signal_shutdown("Sample detected")

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Sample Detector")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    td = SampleDetector()
    td.wait_for_target()
