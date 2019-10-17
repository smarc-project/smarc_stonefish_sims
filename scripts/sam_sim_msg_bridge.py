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
from sam_msgs.msg import ThrusterAngles, ThrusterRPMs

class SAMSimMsgBridge(object):

    def thruster_callback(self, thruster_msg):

	header = Header()
        self.thrusters.publish(header, [thruster_msg.thruster_1_rpm, thruster_msg.thruster_2_rpm])

    def angles_callback(self, angles_msg):

	header = Header()
        thruster_angles = JointState()
        thruster_angles.header = header
        thruster_angles.name = ["sam_auv/joint1", "sam_auv/joint2"]
        thruster_angles.position = [-angles_msg.thruster_horizontal_radians, -angles_msg.thruster_vertical_radians]
        self.joint_states.publish(thruster_angles)

    def __init__(self):
	
        self.joint_states = rospy.Publisher('/sam_auv/desired_joint_states', JointState, queue_size=10)
        self.thrusters = rospy.Publisher('/sam_auv/thruster_setpoints', Setpoints, queue_size=10)

	rospy.Subscriber("/uavcan_rpm_command", ThrusterRPMs, self.thruster_callback)
	rospy.Subscriber("/uavcan_vector_command", ThrusterAngles, self.angles_callback)

if __name__ == "__main__":
    
    rospy.init_node('sam_sim_msg_bridge', anonymous=True)
    bridge = SAMSimMsgBridge()
    rospy.spin()
