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
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from cola2_msgs.msg import Setpoints
from sam_msgs.msg import ThrusterAngles, ThrusterRPMs, PercentStamped

class SAMSimMsgBridge(object):

    def lcg_callback(self, lcg_msg):

	header = Header()
        lcg_pos = JointState()
        lcg_pos.header = header
        lcg_pos.name = [self.robot_name + "/lcg_joint"]
        lcg_pos.position = [self.lcg_joint_min + 0.01*lcg_msg.value*(self.lcg_joint_max-self.lcg_joint_min)]
        self.joint_states.publish(lcg_pos)

    def thruster_callback(self, thruster_msg):

	header = Header()
        self.thrusters.publish(header, [thruster_msg.thruster_1_rpm, thruster_msg.thruster_2_rpm])

    def angles_callback(self, angles_msg):

	header = Header()
        thruster_angles = JointState()
        thruster_angles.header = header
        thruster_angles.name = [self.robot_name + "/joint1", self.robot_name + "/joint2"]
        thruster_angles.position = [-angles_msg.thruster_horizontal_radians, -angles_msg.thruster_vertical_radians]
        self.joint_states.publish(thruster_angles)

    def __init__(self):
	
        self.joint_states = rospy.Publisher('desired_joint_states', JointState, queue_size=10)
        self.thrusters = rospy.Publisher('thruster_setpoints', Setpoints, queue_size=10)
        self.robot_name = rospy.get_param("~robot_name")
        self.lcg_joint_min = rospy.get_param("~lcg_joint_min", -0.01)
        self.lcg_joint_max = rospy.get_param("~lcg_joint_max", 0.01)

	rospy.Subscriber("core/rpm_cmd", ThrusterRPMs, self.thruster_callback)
	rospy.Subscriber("core/thrust_vector_cmd", ThrusterAngles, self.angles_callback)
	rospy.Subscriber("core/lcg_cmd", PercentStamped, self.lcg_callback)

if __name__ == "__main__":
    
    rospy.init_node('sam_sim_msg_bridge', anonymous=True)
    bridge = SAMSimMsgBridge()
    rospy.spin()
