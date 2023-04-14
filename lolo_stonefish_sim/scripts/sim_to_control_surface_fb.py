#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import rospy
from smarc_msgs.msg import ThrusterFeedback, ThrusterRPM
from std_msgs.msg import Float32
from cola2_msgs.msg import Setpoints

"""
A simple node that listens to setpoints sent to lolo and just publishes them
as feedback continously. This is only used for sim that doesnt do the feedback itself and
modifies the angles to the setpoints instantly. So cmd == fb at all times in the sim
"""


def rudder_setpoints_cb(msg):
    global elevon_p, elevon_s, rudder, elevator
    elevon_p = msg.setpoints[3]
    elevon_s = msg.setpoints[4]
    rudder = msg.setpoints[1]
    elevator = msg.setpoints[0]

robot_name = "/lolo"




if __name__ == "__main__":
    rospy.init_node("sim_to_control_surface_fb")
    rate = rospy.Rate(20)

    elevon_p = 0
    elevon_s = 0
    rudder = 0
    elevator = 0

    elevon_p_pub = rospy.Publisher(robot_name+"/core/elevon_port_fb", Float32, queue_size=1)
    elevon_s_pub = rospy.Publisher(robot_name+"/core/elevon_strb_fb", Float32, queue_size=1)
    rudder_pub = rospy.Publisher(robot_name+"/core/rudder_fb", Float32, queue_size=1)
    elevator_pub = rospy.Publisher(robot_name+"/core/elevator_fb", Float32, queue_size=1)
    rudder_setpoints_sub = rospy.Subscriber(robot_name+"/sim/rudder_setpoints", Setpoints, rudder_setpoints_cb, queue_size=1)

    while not rospy.is_shutdown():
        elevon_p_pub.publish(elevon_p)
        elevon_s_pub.publish(elevon_s)
        rudder_pub.publish(rudder)
        elevator_pub.publish(elevator)
        rate.sleep()


