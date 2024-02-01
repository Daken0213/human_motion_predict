#!/usr/bin/env python3
import rospy 
import time
from visualization_msgs.msg import MarkerArray
import numpy as np
import torch
from scipy.spatial.transform import Rotation as R


def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def euler2rot(euler):
    r = R.from_euler('zyx', euler, degrees=True)
    rot = r.as_matrix()
    return rot

def azure2smpl():
    r = R.from_euler('xyz', [90, 0, 90], degrees=True)
    rot = r.as_matrix()
    return rot
rot_azure2smpl = azure2smpl()

azure_joints = [0, 18, 22, 1, 19, 23, 2, 20, 24, 2,
               21, 25, 3, 4, 11, 26, 5, 12, 6, 13, 7, 14]   # azure joint index to smpl joint index

def callback(msg):
    # TODO: predict many people
    # for i in range(len(msg.markers) // 32):    # 32 joints
    if len(msg.markers) > 0:
        smpl_joints = []
        joint0 = np.array([msg.markers[0].pose.position.x, msg.markers[0].pose.position.y, msg.markers[0].pose.position.z])
        euler = quaternion2euler([msg.markers[0].pose.orientation.x, msg.markers[0].pose.orientation.y, msg.markers[0].pose.orientation.z, msg.markers[0].pose.orientation.w])
        rot_camera2pelvis = euler2rot(-euler)
        for item in azure_joints:
            joint = np.array([msg.markers[item].pose.position.x, msg.markers[item].pose.position.y, msg.markers[item].pose.position.z])
            joint = joint - joint0
            joint_smpl = np.matmul(rot_azure2smpl, np.matmul(rot_camera2pelvis, joint))
            smpl_joints.append(joint_smpl)
        input_data.append(smpl_joints)
        if len(input_data) == 50:
            motion_input = torch.FloatTensor(input_data).cuda()
            motion_input = motion_input[:,4:].reshape(1, 50, -1)
            
            print(motion_input.shape)
            input_data.clear()
            
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/body_tracking_data", MarkerArray, callback)
    rospy.spin()
    
if __name__ == '__main__':
    input_data = []
    listener()