#!/usr/bin/python

# Copyright 2018 Nils Bore (nbore@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import JointState, BatteryState, FluidPressure
from cola2_msgs.msg import Setpoints
from sam_msgs.msg import ThrusterAngles, ThrusterRPMs, PercentStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion 
import numpy as np 
from nav_msgs.msg import Odometry
import message_filters 
import tf

class SAMSimMsgBridge(object):
    
    def sbg_callback(self, imu_msg, odom_msg):
        enu_imu_msg = imu_msg

        # Using the Yaw from the GT SAM as compass heading
        (roll_gt, pitch_gt, yaw_gt) = tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                                                              odom_msg.pose.pose.orientation.y,
                                                                              odom_msg.pose.pose.orientation.z, 
                                                                              odom_msg.pose.pose.orientation.w])

        # Conversion to ENU
        (roll_sbg, pitch_sbg, yaw_sbg) = tf.transformations.euler_from_quaternion([imu_msg.orientation.y,
                                                                              imu_msg.orientation.x,
                                                                              -imu_msg.orientation.z, 
                                                                              imu_msg.orientation.w])

        quat_rot = tf.transformations.quaternion_from_euler(pitch_sbg, roll_sbg+3.14, yaw_gt)
        enu_imu_msg.orientation = Quaternion(quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3])  

        enu_imu_msg.angular_velocity.x = imu_msg.angular_velocity.y                                         
        enu_imu_msg.angular_velocity.y = imu_msg.angular_velocity.x                                         
        enu_imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z

        enu_imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y                                     
        enu_imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.x                                         
        enu_imu_msg.linear_acceleration.z = -imu_msg.linear_acceleration.z 

        self.sbg_pub.publish(enu_imu_msg)
        
    def stim_callback(self, imu_msg):
        enu_imu_msg = imu_msg

        # Conversion to ENU
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([imu_msg.orientation.y,
                                                                              imu_msg.orientation.x,
                                                                              -imu_msg.orientation.z, 
                                                                              imu_msg.orientation.w])

        quat_rot = tf.transformations.quaternion_from_euler(pitch, roll+3.14, yaw)
        enu_imu_msg.orientation = Quaternion(quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3])  

        enu_imu_msg.angular_velocity.x = imu_msg.angular_velocity.y                                         
        enu_imu_msg.angular_velocity.y = imu_msg.angular_velocity.x                                         
        enu_imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z

        enu_imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y                                     
        enu_imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.x                                         
        enu_imu_msg.linear_acceleration.z = -imu_msg.linear_acceleration.z 

        self.stim_pub.publish(enu_imu_msg)

    def press_callback(self, press_msg):

        press_msg.fluid_pressure += 101325.0
        self.press_pub.publish(press_msg)

    def vbs_callback(self, vbs_msg):

        self.vbs_pub.publish(self.vbs_vol_min + 0.01*vbs_msg.value*(self.vbs_vol_max - self.vbs_vol_min))

    def vbs_vol_callback(self, vol_msg):

	header = Header()
        self.vbs_fb_pub.publish(100.*(.5+vol_msg.data), header)

    def joint_state_callback(self, msg):

        if msg.name[0] == "sam/lcg_joint":
            header = Header()
            pos = 100./(self.lcg_joint_max-self.lcg_joint_min)*(msg.position[0] - self.lcg_joint_min)
            self.lcg_fb_pub.publish(pos, header)

    def lcg_callback(self, lcg_msg):

	header = Header()
        lcg_pos = JointState()
        lcg_pos.header = header
        lcg_pos.name = [self.robot_name + "/lcg_joint"]
        lcg_pos.position = [self.lcg_joint_min + 0.01*lcg_msg.value*(self.lcg_joint_max-self.lcg_joint_min)]
        self.joint_states.publish(lcg_pos)

    def thruster_callback(self, thruster_msg):

	header = Header()
        self.last_thruster_msg = Setpoints(header, [thruster_msg.thruster_1_rpm, thruster_msg.thruster_2_rpm])
        self.last_thruster_msg_time = rospy.get_time()

    def publish_thruster_callback(self, event):

        current_time = rospy.get_time()
        # must publish at 10Hz to get the thrusters going
        if current_time - self.last_thruster_msg_time < 0.1:
            self.thrusters.publish(self.last_thruster_msg)
        else:
            self.thrusters.publish(Header(), [0., 0.])

    def angles_callback(self, angles_msg):

	header = Header()
        thruster_angles = JointState()
        thruster_angles.header = header
        thruster_angles.name = [self.robot_name + "/thruster_yaw_joint", self.robot_name + "/thruster_pitch_joint"]
        thruster_angles.position = [angles_msg.thruster_horizontal_radians, angles_msg.thruster_vertical_radians]
        self.joint_states.publish(thruster_angles)

    def battery_callback(self, event):

        self.battery_msg.header.stamp = rospy.Time.now()
        self.battery_pub.publish(self.battery_msg)

    def __init__(self):
	
        self.robot_name = rospy.get_param("~robot_name")
        self.lcg_joint_min = rospy.get_param("~lcg_joint_min", -0.01)
        self.lcg_joint_max = rospy.get_param("~lcg_joint_max", 0.01)
        self.vbs_vol_min = rospy.get_param("~vbs_vol_min", -0.5)
        self.vbs_vol_max = rospy.get_param("~vbs_vol_max", 0.5)
        self.last_thruster_msg_time = 0.
        self.last_thruster_msg = Setpoints(Header(), [0., 0.])

        self.joint_states = rospy.Publisher('desired_joint_states', JointState, queue_size=10)
        self.thrusters = rospy.Publisher('thruster_setpoints', Setpoints, queue_size=10)
        self.vbs_pub = rospy.Publisher('vbs/setpoint', Float64, queue_size=10)
        self.vbs_fb_pub = rospy.Publisher('core/vbs_fb', PercentStamped, queue_size=10)
        self.lcg_fb_pub = rospy.Publisher('core/lcg_fb', PercentStamped, queue_size=10)
        self.battery_pub = rospy.Publisher('core/battery_fb', BatteryState, queue_size=10)
        self.press_pub = rospy.Publisher('core/depth20_pressure', FluidPressure, queue_size=10)
        self.sbg_pub = rospy.Publisher('core/sbg_imu', Imu, queue_size=10)
        self.stim_pub = rospy.Publisher('core/stim_imu', Imu, queue_size=10)

	rospy.Subscriber("core/rpm_cmd", ThrusterRPMs, self.thruster_callback)
	rospy.Subscriber("core/thrust_vector_cmd", ThrusterAngles, self.angles_callback)
	rospy.Subscriber("core/lcg_cmd", PercentStamped, self.lcg_callback)
	rospy.Subscriber("core/vbs_cmd", PercentStamped, self.vbs_callback)
        rospy.Subscriber("vbs/volume_centered", Float64, self.vbs_vol_callback)
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("core/depth20_pressure_sim", FluidPressure, self.press_callback)
        #  rospy.Subscriber("core/sbg_imu_sim", Imu, self.sbg_callback)
        rospy.Subscriber("core/stim_imu_sim", Imu, self.stim_callback)

        self.subs_sbg = message_filters.Subscriber('core/sbg_imu_sim', Imu)
        self.subs_odom_gt = message_filters.Subscriber('gt/odom', Odometry)  
        self.ts = message_filters.ApproximateTimeSynchronizer([self.subs_sbg, self.subs_odom_gt],
                                                          50, slop=50.0, allow_headerless=True)
        self.ts.registerCallback(self.sbg_callback)

        self.battery_msg = BatteryState()
        self.battery_msg.voltage = 12.5
        self.battery_msg.percentage = 81.
        self.battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery_msg.header = Header()

        rospy.Timer(rospy.Duration(1), self.battery_callback)
        rospy.Timer(rospy.Duration(0.1), self.publish_thruster_callback)

if __name__ == "__main__":
    
    rospy.init_node('sam_sim_msg_bridge', anonymous=True)
    bridge = SAMSimMsgBridge()
    rospy.spin()
