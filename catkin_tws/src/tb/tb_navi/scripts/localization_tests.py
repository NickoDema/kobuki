#!/usr/bin/env python

##   localization_tests.py
##   Maintainer: Nikoaly Dema
##        Email: ndema2301@gmail.com
##        goTRG | spb


import rospy
import tf

from math import fabs, sqrt

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped


class LocalizationTester():
    def __init__(self):

        self.path_from_odometry = Path()
        self.path_from_localization = Path()

        self.path_from_odometry.header.frame_id = "map"
        self.path_from_localization.header.frame_id = "map"

        self.max_point_cont = 1000
        self.min_lin_diff = 0.05 # in meters
        self.min_ang_diff = 0.05 # ~3 degree

        self.tf_rate = 0.1

        rospy.init_node('localization_test', anonymous=True)

        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Timer(rospy.Duration(self.tf_rate), self.localization_cb)

        self.odom_path_pub = rospy.Publisher('odom_path', Path, queue_size=1)
        self.locz_path_pub = rospy.Publisher('localization_path', Path, queue_size=1)

        self.tf_listener = tf.TransformListener()


    def check_diff(self, x, y, q, prev_x, prev_y, prev_q):

        lin_diff = sqrt((x - prev_x)**2 + (y - prev_y)**2)

        euler = tf.transformations.euler_from_quaternion(q)
        prev_euler = tf.transformations.euler_from_quaternion(prev_q)
        ang_diff = euler[2] - prev_euler[2]

        if (fabs(ang_diff) < self.min_ang_diff) and (lin_diff < self.min_lin_diff):
            return False

        return True


    def add_to_path(self, path, publisher, x, y, q):

        if path.poses:
            prev = path.poses[-1]
            prev_x = prev.pose.position.x
            prev_y = prev.pose.position.y
            prev_q = [0, 0, prev.pose.orientation.z, prev.pose.orientation.w]

            if not self.check_diff(x, y, q, prev_x, prev_y, prev_q):
                return

            if len(path.poses) == self.max_point_cont:
                path.poses.pop(0)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.seq += 1
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        path.poses.append(pose_msg)
        path.header.stamp = pose_msg.header.stamp
        publisher.publish(path)


    def localization_cb(self, event):

        try:
            (pos, q) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            x = pos[0]
            y = pos[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("[AGV]: Can\'t get map - base_link transform")
            rospy.sleep(2.)
            return

        self.add_to_path(self.path_from_localization, self.locz_path_pub, x, y, q)


    def odom_cb(self, odom_msg):

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = [0, 0, odom_msg.pose.pose.orientation.z,
                   odom_msg.pose.pose.orientation.w]

        self.add_to_path(self.path_from_odometry, self.odom_path_pub, x, y, q)


if __name__ == '__main__':
    tester = LocalizationTester()
    rospy.spin()
