#! /usr/bin/env python3

"""
We want to simulate the INS driver in the simulation.
Stonefish has no INS by itself, so we'll fake it by fusing GPS, IMU etc into the same
message the real INS driver would produce.
Any dead-reckoning package should read this INS message and publish lolo/base_link -> map connections in TF
"""

import rospy, tf
from ixblue_ins_msgs.msg import Ins
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import PoseStamped, Point
from smarc_msgs.srv import UTMToLatLon
from nav_msgs.msg import Odometry
# Ins message:
# int8 ALT_REF_GEOID=0
# int8 ALT_REF_ELLIPSOID=1
# std_msgs/Header header
  # uint32 seq
  # time stamp
  # string frame_id
# float64 latitude
# float64 longitude
# int8 altitude_ref
# float32 altitude
# float64[9] position_covariance
# float32 heading
# float32 roll
# float32 pitch
# float64[9] attitude_covariance
# geometry_msgs/Vector3 speed_vessel_frame
  # float64 x
  # float64 y
  # float64 z
# float64[9] speed_vessel_frame_covariance


class INS(object):
    def __init__(self,
                 robot_name="lolo"):

        odom_topic = "/"+robot_name+"/sim/odom"
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=1)

        # the usual structure of TF is:
        # UTM -> world_ned -> gt/base_link -> base_link
        self.base_link = robot_name+"/base_link"
        self.gt_base_link = "gt/"+self.base_link
        self.world_link = "world_ned"
        self.utm_link = "utm"

        self.tf_listener = tf.TransformListener()

        # used for turning the utm pose of gt into lat/lon
        self.utm2ll_service_name = '/'+robot_name+"/dr/utm_to_lat_lon"
        while True:
            try:
                rospy.wait_for_service(self.utm2ll_service_name, timeout=5)
                break
            except:
                rospy.logwarn("Waiting for dr/utm_to_lat_lon service until it is available")
        self.utm2ll_service = rospy.ServiceProxy(self.utm2ll_service_name, UTMToLatLon)


        self.ins_msg = Ins()
        self.ins_msg.header.frame_id = self.base_link
        self.ins_msg.altitude_ref = Ins.ALT_REF_GEOID

        self.ins_pub = rospy.Publisher("/"+robot_name+"/core/ins", Ins, queue_size=1)


    def odom_cb(self, msg):
        """
        move the velocities from odom to ins
        odom is in base_link already, and since we are using gt stuff, we dont need to transform anything
        """
        # this altitude is altitude in the global sense, from sea level!
        self.ins_msg.altitude = -msg.pose.pose.position.z
        self.ins_msg.speed_vessel_frame.x = msg.twist.twist.linear.x
        self.ins_msg.speed_vessel_frame.y = msg.twist.twist.linear.y
        self.ins_msg.speed_vessel_frame.z = msg.twist.twist.linear.z



    def update_from_gt(self):
        self.tf_listener.waitForTransform(self.utm_link,
                                          self.gt_base_link,
                                          rospy.Time(0),
                                          rospy.Duration(5))

        gt_posi, gt_ori_quat = self.tf_listener.lookupTransform(self.utm_link,
                                                                self.gt_base_link,
                                                                rospy.Time(0))
        # position is in UTM meters. need lat lon from it
        # service wants a Point input, gt_posi is a list
        p = Point()
        p.x = gt_posi[0]
        p.y = gt_posi[1]
        p.z = 0
        res = self.utm2ll_service(p)
        self.ins_msg.latitude = res.lat_lon_point.latitude
        self.ins_msg.longitude = res.lat_lon_point.longitude


        # turn that quat into rpy
        rpy = tf.transformations.euler_from_quaternion(gt_ori_quat)
        self.ins_msg.roll = rpy[0]
        self.ins_msg.pitch = rpy[1]
        self.ins_msg.heading = rpy[2]


    def publish(self):
        self.update_from_gt()
        self.ins_msg.header.seq += 1
        self.ins_msg.header.stamp = rospy.Time.now()
        self.ins_pub.publish(self.ins_msg)



if __name__ == "__main__":
    rospy.init_node("sim_to_ins")

    ins = INS()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        ins.publish()
        rate.sleep()




