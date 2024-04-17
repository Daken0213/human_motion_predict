#!/usr/bin/env python3
import rospy
import tf
import tf.msg
import geometry_msgs.msg
from airobots_calvin.msg import MotorStatus
import math
import threading

class HeadmotorListener:
    def __init__(self):
        self.listener = rospy.Subscriber("/headmotor/motorstatus", MotorStatus, self.callback)
        self.head1_angle = 0.0
        self.head2_angle = 0.0
    def callback(self, msg):
        shared_resource_lock.acquire()
        for item in msg.status:
            if item.id == 9:
                self.head1_angle = item.angle
            elif item.id == 8:
                self.head2_angle = item.angle
            else:
                rospy.logerr("Unknown motor id")
        shared_resource_lock.release()

class DynamicTFBroadcaster:
    def __init__(self):
        self.br_head1 = tf.TransformBroadcaster()
        self.br_head2 = tf.TransformBroadcaster()
        self.headmotor_listener = HeadmotorListener()        
        
        while not rospy.is_shutdown():
            shared_resource_lock.acquire()
            head1_radian = math.radians(self.headmotor_listener.head1_angle)
            head2_radian = math.radians(self.headmotor_listener.head2_angle)
            shared_resource_lock.release()
            
            t1 = geometry_msgs.msg.TransformStamped()
            t1.header.stamp = rospy.Time.now()
            t1.header.frame_id = "base"
            t1.child_frame_id = "head1"
            t1.transform.translation.x = 0.01051
            t1.transform.translation.y = 0.0
            t1.transform.translation.z = 0.2048
            t1.transform.rotation.x = 0.0
            t1.transform.rotation.y = 0.0
            t1.transform.rotation.z = math.sin(head1_radian / 2)
            t1.transform.rotation.w = math.cos(head1_radian / 2)
            self.br_head1.sendTransformMessage(t1)
            
            t2 = geometry_msgs.msg.TransformStamped()
            t2.header.stamp = rospy.Time.now()
            t2.header.frame_id = "head1"
            t2.child_frame_id = "head2"
            t2.transform.translation.x = 0.0
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.0729366
            t2.transform.rotation.x = math.sin(-0.25 * math.pi) * math.cos(0.5 * math.pi - head2_radian / 2)
            t2.transform.rotation.y = math.sin(-0.25 * math.pi) * math.sin(0.5 * math.pi - head2_radian / 2)
            t2.transform.rotation.z = math.cos(-0.25 * math.pi) * math.sin(0.5 * math.pi + head2_radian / 2)
            t2.transform.rotation.w = math.cos(-0.25 * math.pi) * math.cos(0.5 * math.pi + head2_radian / 2)
            self.br_head2.sendTransformMessage(t2)
            
            rate.sleep()

if __name__ == '__main__':
    shared_resource_lock = threading.Lock()
    rospy.init_node('tf_broadcaster')
    rate = rospy.Rate(30)
    tfb = DynamicTFBroadcaster()
    rospy.spin()
    
    