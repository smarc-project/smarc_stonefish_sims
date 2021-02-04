#!/usr/bin/env python

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

import pygame
from pygame.constants import K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s
import rospy
#from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from cola2_msgs.msg import Setpoints
from sam_msgs.msg import ThrusterAngles
from smarc_msgs.msg import ThrusterRPM

class TeleopServer(object):

    def callback(self, image_msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        except CvBridgeError as e:
            print(e)

        self.surface = pygame.image.frombuffer(cv_image.tostring(), cv_image.shape[:2], "RGB")

    def __init__(self):

        rospy.init_node('keyboard_teleop', anonymous=True)
        pygame.init()

        self.surface = None
        self.bridge = CvBridge()

        #self.joint_states = rospy.Publisher('/sam_auv/desired_joint_states', JointState, queue_size=10)
        #self.thrusters = rospy.Publisher('/sam_auv/thruster_setpoints', Setpoints, queue_size=10)
        self.rudder_angles = rospy.Publisher('core/rudder_cmd', Float32, queue_size=10)
        self.elevator_angle = rospy.Publisher('core/elevator_cmd', Float32, queue_size=10)
        self.elevon_stbd_angle = rospy.Publisher('core/elevon_strb_cmd', Float32, queue_size=10)
        self.elevon_port_angle = rospy.Publisher('core/elevon_port_cmd', Float32, queue_size=10)
        self.thruster1_rpms = rospy.Publisher('core/thruster1_cmd', ThrusterRPM, queue_size=10)
        self.thruster2_rpms = rospy.Publisher('core/thruster2_cmd', ThrusterRPM, queue_size=10)

        rospy.Subscriber("/sam_auv/camera_thruster/camera_image", Image, self.callback)

        screen = pygame.display.set_mode((200, 200))
        pygame.display.flip()
        header = Header()
        #thruster_angles = JointState()
        #thruster_angles.header = header
        #thruster_angles.name = ["sam_auv/joint1", "sam_auv/joint2"]

        joint_angle = 0.62
        thrust_level = 2000

        clock = pygame.time.Clock()
        while not rospy.is_shutdown():

            if self.surface is not None:
                screen.blit(self.surface, (0, 0))
            pygame.display.update()

            keys = pygame.key.get_pressed()
            self.joint_z_angle = 0. # top
            self.joint_y_angle = 0. # left
            thrust = 0.

            if keys[K_LEFT]:
                self.joint_z_angle = -joint_angle
            if keys[K_RIGHT]:
                self.joint_z_angle = joint_angle
            if keys[K_UP]:
                self.joint_y_angle = joint_angle
            if keys[K_DOWN]:
                self.joint_y_angle = -joint_angle
            if keys[K_w]:
                self.thruster1_rpms.publish(thrust_level)
                self.thruster2_rpms.publish(thrust_level)
            if keys[K_s]:
                self.thruster1_rpms.publish(0.)
                self.thruster2_rpms.publish(0.)

            #thruster_angles.position = [self.joint_z_angle, self.joint_y_angle]
            #self.joint_states.publish(thruster_angles)
            #self.thruster_angles.publish(self.joint_y_angle, self.joint_z_angle, header)
            self.rudder_angles.publish(self.joint_z_angle)
            self.elevator_angle.publish(self.joint_y_angle)
            self.elevon_port_angle.publish(-self.joint_y_angle)
            self.elevon_stbd_angle.publish(self.joint_y_angle)

            pygame.event.pump()
            clock.tick(20)

if __name__ == "__main__":
    
    teleop = TeleopServer()
