#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import tf
from tf.transformations import quaternion_matrix
import math
import numpy as np
import threading

class BodyListener:
    def __init__(self, topic_name):
        self.listener = rospy.Subscriber(topic_name, MarkerArray, self.callback)
        self.tf_listener = tf.TransformListener()
        self.body_index = [0, 5, 12, # trunk
                           6, 7,    # left arm
                           13, 14,  # right arm
                           18, 19, 20,  # left leg 21
                           22, 23, 24]  # right leg 25  # 26 # head 
        self.body_joints = []
        self.trans = []
        self.rot = []
    def callback(self, msg):
        self.body_joints = []
        shared_resource_lock.acquire()
        if len(msg.markers) > 0:
            for item in self.body_index:
                joint = np.array([msg.markers[item].pose.position.x, msg.markers[item].pose.position.y, msg.markers[item].pose.position.z])
                self.body_joints.append(joint)
        try:
            (self.trans, self.rot) = self.tf_listener.lookupTransform('/base', '/depth_camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        shared_resource_lock.release()
    
class BodyShapeMarker:
    def __init__(self, mark_topic_name, listen_topic_name):
        self.body_listener = BodyListener(listen_topic_name)
        self.body_pub= rospy.Publisher(mark_topic_name, MarkerArray, queue_size=1)
        self.maker_array = MarkerArray()
        self.body_joints = []
        
    def depth2base(self):
        self.body_joints = []
        shared_resource_lock.acquire()
        for joint in self.body_listener.body_joints:
            self.body_joints.append(np.matmul(quaternion_matrix(self.body_listener.rot)[:3, :3], joint) + self.body_listener.trans)
        shared_resource_lock.release()
    
    def calVec2Quat(self, vec):
        psi = math.atan2(vec[1], vec[0])
        theta = math.acos(vec[2] / np.linalg.norm(vec))
        qx = -math.sin(theta / 2) * math.sin(psi / 2)
        qy = math.sin(theta / 2) * math.cos(psi / 2)
        qz = math.cos(theta / 2) * math.sin(psi / 2)
        qw = math.cos(theta / 2) * math.cos(psi / 2)
        return [qx, qy, qz, qw]
    
    def markCylider(self, id, start, end, diameter):
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CYLINDER
        marker.id = id
        marker.action = marker.ADD
        
        marker.scale.x = diameter
        marker.scale.y = marker.scale.x
        marker.scale.z = np.linalg.norm(start - end)
        
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        position = (start + end) / 2
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        orientation = self.calVec2Quat(start - end)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        self.maker_array.markers.append(marker)
        
    def markTrunk(self):
        start = self.body_joints[0]
        end = (self.body_joints[1] + self.body_joints[2]) / 2
        diameter = np.linalg.norm(self.body_joints[1] - self.body_joints[2])
        self.markCylider(0, start, end, diameter)
        
    def markLimbs(self):
        arm_diameter = 0.1
        leg_diameter = 0.17
        self.markCylider(1, self.body_joints[1], self.body_joints[3], arm_diameter) # upper left arm
        self.markCylider(2, self.body_joints[3], self.body_joints[4], arm_diameter) # lower left arm
        self.markCylider(3, self.body_joints[2], self.body_joints[5], arm_diameter) # upper right arm
        self.markCylider(4, self.body_joints[5], self.body_joints[6], arm_diameter) # lower right arm
        self.markCylider(5, self.body_joints[7], self.body_joints[8], leg_diameter) # upper left leg
        self.markCylider(6, self.body_joints[8], self.body_joints[9], leg_diameter) # lower left leg
        self.markCylider(7, self.body_joints[10], self.body_joints[11], leg_diameter)   # upper right leg
        self.markCylider(8, self.body_joints[11], self.body_joints[12], leg_diameter)   # lower right leg
        
    def mark(self):
        self.maker_array = MarkerArray()
        self.depth2base()
        if len(self.body_joints) > 0:
            self.markTrunk()
            self.markLimbs()
        self.body_pub.publish(self.maker_array)
        
if __name__ == '__main__':
    rospy.init_node('body_shape_marker')
    rate = rospy.Rate(30)
    shared_resource_lock = threading.Lock()
    bsm_now = BodyShapeMarker("/body", "/body_tracking_data")   # mark body shape now
    while not rospy.is_shutdown():
        bsm_now.mark()
        rate.sleep()
    rospy.spin()