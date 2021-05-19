#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


class sim_sss_detector:
    def __init__(self):
        self.prev_pose = None
        self.current_pose = None
        self.marked_positions = None

        self.odom_sub = rospy.Subscriber('/sam/dr/odom', Odometry,
                                         self._update_pose)
        self.marked_pos_sub = rospy.Subscriber('/sam/sim/marked_positions',
                                               MarkerArray,
                                               self._update_marked_positions)
        self.pub = rospy.Publisher('/sam/sim/sidescan/detection',
                                   Detection2DArray,
                                   queue_size=2)

    #TODO: get things into the correct frame. marked_positions is published in map frame,
    #      dr/odom is in world_ned
    def _update_marked_positions(self, msg):
        self.marked_positions = msg

    def _update_pose(self, msg):
        """Update prev_pose and current_pose according to the odom msg received"""
        if not self.prev_pose:
            self.prev_pose = msg.pose.pose
            self.current_pose = msg.pose.pose

        self.prev_pose = self.current_pose
        self.current_pose = msg.pose.pose

        heading = self.calculate_heading()
        print(heading)

    def calculate_heading(self):
        """Returns a unit vector of heading based on the difference between
        current_pose and prev_pose"""
        if not self.prev_pose or not self.current_pose:
            raise rospy.ROSException(
                'Not enough odom measurement for heading calculation')

        dx = self.current_pose.position.x - self.prev_pose.position.x
        dy = self.current_pose.position.y - self.prev_pose.position.y
        dz = self.current_pose.position.z - self.prev_pose.position.z

        # Normalization
        heading = np.array([dx, dy, dz]).reshape(-1, 1)
        norm = np.linalg.norm(heading)
        if norm > 0:
            heading = heading / norm

        return heading


def main():
    rospy.init_node('sim_sss_detection_publisher', anonymous=False)
    rospy.Rate(5)  # ROS Rate at 5Hz

    detector = sim_sss_detector()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
