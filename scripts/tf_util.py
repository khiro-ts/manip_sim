import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped

def tf2hom(tf_in):
    tfeu = tft.euler_from_quaternion([tf_in.rotation.x, tf_in.rotation.y, tf_in.rotation.z, tf_in.rotation.w],axes='sxyz')
    tftr = [tf_in.translation.x, tf_in.translation.y, tf_in.translation.z]
    M = tft.compose_matrix(angles=tfeu, translate=tftr)

    return M

def hom2tf(Mat):
    scale, shear, angles, trans, persp = tft.decompose_matrix(Mat)
    quat = tft.quaternion_from_euler(angles[0], angles[1], angles[2])
    tfo  = Transform()
    tfo.rotation.x = quat[0]
    tfo.rotation.y = quat[1]
    tfo.rotation.z = quat[2]
    tfo.rotation.w = quat[3]
    tfo.translation.x = trans[0]
    tfo.translation.y = trans[1]
    tfo.translation.z = trans[2]
    return tfo

def tf_diff(tf1, tf2):
    M1 = tf2hom(tf1)
    M2 = tf2hom(tf2)
    return hom2tf(M2.dot(tft.inverse_matrix(M1)))
    
def pose2hom(pose_in: Pose):
    try:
        tfeu = tft.euler_from_quaternion([pose_in.rotation.x, pose_in.rotation.y, pose_in.rotation.z, pose_in.rotation.w],axes='sxyz')
        tftr = [pose_in.translation.x, pose_in.translation.y, pose_in.transation.z]
        M = tft.compose_matrix(angles=tfeu, translate=tftr)
        return M
    except:
        print("Input must be a pose object")

def hom2pose(Mat):
    scale, shear, angles, trans, persp = tft.decompose_matrix(Mat)
    quat = tft.quaternion_from_euler(angles[0], angles[1], angles[2])
    pose  = Pose()
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    return pose 

def pose_diff(p1, p2):
    M1 = pose2hom(p1)
    M2 = pose2hom(p2)
    return hom2pose(M2.dot(tft.inverse_matrix(M1)))

def transform_pose(tf_buffer, source_frame, target_frame, pose_in):
    try:
    
        trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())

        tr = trans.transform.translation
        rot = trans.transform.rotation
        
        vtr = [tr.x, tr.y, tr.z]
        quat = [rot.x, rot.y, rot.z, rot.w]

        pose_out = PoseStamped()
        pose_out.header.frame_id = target_frame
        pose_out.pose.position = tft.quaternion_multiply(
            tft.quaternion_multiply([pose_in.transform.translation.x, pose_in.transform.translation.y, pose_in.transform.translation.z, 0.0], quat),
            tft.quaternion_conjugate([vtr[0], vtr[1], vtr[2], 0.0])
        )[:3]
        pose_out.pose.orientation = tft.quaternion_multiply([pose_in.transform.rotation.x, pose_in.transform.rotation.y, pose_in.transform.rotation.z, pose_in.transform.rotation.w], 
                                                            quat)
        
        return pose_out
    except:
        rospy.logerr("tf transform error: {e}")
        return None