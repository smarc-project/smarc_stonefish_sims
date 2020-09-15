#!/usr/bin/python

import rospy
import numpy as np
from sam_msgs.msg import ThrusterRPMs
from geometry_msgs.msg import TwistStamped
from uavcan_ros_bridge.msg import ESCStatus
from sbg_driver.msg import SbgEkfEuler
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Quaternion

class GTOdom(object):

    def __init__(self):
        
        self.rpm_fb_topic = rospy.get_param('~thrust_fb', '/sam/core/rpm_fb')
        self.gt_odom_top = rospy.get_param('~gt_odom_top', '/sam/gt/odom')
        self.base_frame = rospy.get_param('~base_frame', 'sam/base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.sim_odom_top = rospy.get_param('~sim_odom_top', '/sam/dynamics/odometry')

        self.sub_stone_odom = rospy.Subscriber(self.sim_odom_top, Odometry, self.gt_odom_cb)

        self.pub_odom = rospy.Publisher(self.gt_odom_top, Odometry, queue_size=10)
       
        rospy.spin()


    def gt_odom_cb(self, stone_odom_msg):

        odom_msg = Odometry()
        odom_msg.header.frame_id = self.map_frame
        odom_msg.header.stamp = stone_odom_msg.header.stamp 
        odom_msg.child_frame_id = 'gt/'+ self.base_frame
        odom_msg.pose.pose.position.x = stone_odom_msg.pose.pose.position.y
        odom_msg.pose.pose.position.y =  stone_odom_msg.pose.pose.position.x
        odom_msg.pose.pose.position.z = -stone_odom_msg.pose.pose.position.z
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([stone_odom_msg.pose.pose.orientation.y,
                                                                       stone_odom_msg.pose.pose.orientation.x,
                                                                       -stone_odom_msg.pose.pose.orientation.z,
                                                                       stone_odom_msg.pose.pose.orientation.w])
        
        quat = tf.transformations.quaternion_from_euler(pitch, roll+3.14, yaw-1.57)            
        odom_msg.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        self.pub_odom.publish(odom_msg)

        #  br = tf.TransformBroadcaster()
        #  position_t = [odom_msg.pose.pose.position.x,
                     #  odom_msg.pose.pose.position.y,
                     #  odom_msg.pose.pose.position.z]
        #  br.sendTransform(position_t, quat, rospy.Time.now(), "sam_test", self.map_frame)

if __name__ == "__main__":

    rospy.init_node('gt_odom_provider')
    try:
        GTOdom()
    except rospy.ROSInterruptException:
        pass
