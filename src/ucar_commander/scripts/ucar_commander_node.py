#! /usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum
from threading import Timer
from time import sleep
from playsound import playsound

import rospy
import actionlib
import os
import tf
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int8
from std_srvs.srv import SetBool
from qr_msgs.srv import qr as QRsrv

from threading import Timer, Lock


class NavStage(Enum):
    IDLE = 0
    TO_SCAN = 1
    ENROUTE_TO_SCAN = 2
    SCAN_CODE = 3
    TO_DESTINATION = 4
    ENROUTE_TO_DESTINATION = 5


class UcarCommander(object):

    def __init__(self):
        # ROS Infastructure
        rospy.init_node('ucar_commander_node')
        self.tf_listener = tf.TransformListener()
        self.pub_scan_region = rospy.Publisher(
            "scan_region", PolygonStamped, queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.srv_nav_start = rospy.Service(
            'nav_start', SetBool, self.nav_start_srv_handler)

        # vars
        self.is_nav_start = False  # will be fliped to true, when service get called
        self.nav_stage = NavStage.IDLE
        self.position = Point32()
        self.global_frame_id = "map"
        self.robot_frame_id = "base_link"

        self.scan_region = PolygonStamped()
        self.scan_goal = MoveBaseGoal()
        self.end_goal = MoveBaseGoal()

        park_1 = MoveBaseGoal()
        park_2 = MoveBaseGoal()
        park_3 = MoveBaseGoal()

        self.scan_goal.target_pose.header.frame_id = self.global_frame_id
        self.scan_goal.target_pose.pose.position.x = 2.867
        self.scan_goal.target_pose.pose.position.y = -3.612
        self.scan_goal.target_pose.pose.position.z = 0
        self.scan_goal.target_pose.pose.orientation.x = 0.000
        self.scan_goal.target_pose.pose.orientation.y = 0.000
        self.scan_goal.target_pose.pose.orientation.z = -1
        self.scan_goal.target_pose.pose.orientation.w = 0

        park_1.target_pose.header.frame_id = self.global_frame_id
        park_1.target_pose.pose.position.x = -0.128
        park_1.target_pose.pose.position.y = -1.408
        park_1.target_pose.pose.position.z = 0
        park_1.target_pose.pose.orientation.x = 0.000
        park_1.target_pose.pose.orientation.y = 0.000
        park_1.target_pose.pose.orientation.z = 0.695
        park_1.target_pose.pose.orientation.w = 0.719

        park_2.target_pose.header.frame_id = self.global_frame_id
        park_2.target_pose.pose.position.x = -0.128
        park_2.target_pose.pose.position.y = -1.408
        park_2.target_pose.pose.position.z = 0
        park_2.target_pose.pose.orientation.x = 0.000
        park_2.target_pose.pose.orientation.y = 0.000
        park_2.target_pose.pose.orientation.z = 0.695
        park_2.target_pose.pose.orientation.w = 0.719

        park_3.target_pose.header.frame_id = self.global_frame_id
        park_3.target_pose.pose.position.x = -0.128
        park_3.target_pose.pose.position.y = -1.408
        park_3.target_pose.pose.position.z = 0
        park_3.target_pose.pose.orientation.x = 0.000
        park_3.target_pose.pose.orientation.y = 0.000
        park_3.target_pose.pose.orientation.z = 0.695
        park_3.target_pose.pose.orientation.w = 0.719

        self.destinations = [park_1, park_2, park_3]

        _scan_region = [[2.173, -3.406], [2.173, -2.768],
                        [3.416, -2.768], [3.416, -3.406]]

        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()
        pt1.x = _scan_region[0][0]
        pt1.y = _scan_region[0][1]
        pt2.x = _scan_region[1][0]
        pt2.y = _scan_region[1][1]
        pt3.x = _scan_region[2][0]
        pt3.y = _scan_region[2][1]
        pt4.x = _scan_region[3][0]
        pt4.y = _scan_region[3][1]

        self.scan_region.header.frame_id = self.global_frame_id
        self.scan_region.polygon.points.append(pt1)
        self.scan_region.polygon.points.append(pt2)
        self.scan_region.polygon.points.append(pt3)
        self.scan_region.polygon.points.append(pt4)

    def nav_start_srv_handler(self, req):
        if not self.is_nav_start:
            if req.data:
                print("Action Stations!!!")
                self.nav_stage = NavStage.TO_SCAN
                self.is_nav_start = True
                return True, "Started, good luck"

    def code_scan_srv_client(request):
        try:
            sp = rospy.ServiceProxy('/qr/scan_qr', QRsrv)
            response = sp(True)
            if response.success:
                return response.id.data
            else:
                return -1
        except rospy.ServiceException as ex:
            print("Service call to scan QR code failed: " + str(ex))
            return -1

    def _scan_qr_code(self):
        id = self.code_scan_srv_client()
        print("scan")
        if id == -1:
            pass
        elif id >= 0 and id <= 2:
            print("Got QR code: " + str(id))
            self.end_goal = self.destinations[id]
            self.nav_stage = NavStage.TO_DESTINATION
        else:
            rospy.logwarn("ID out of range " + str(id))
            rospy.logwarn("It might be a miss detect or you encountered a bug")

    def _get_robot_pose(self):
        # lookup tf and get robot pose in map frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            self.position.x = trans[0]
            self.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup failed")

    def is_in_region(self, trigger_region):
        # check is robot in trigger region
        x_min = trigger_region.polygon.points[0].x
        x_max = trigger_region.polygon.points[0].x
        y_min = trigger_region.polygon.points[0].y
        y_max = trigger_region.polygon.points[0].y
        # find the x_min, y_min, x_max, y_max of the trigger region
        for point in trigger_region.polygon.points:
            if point.x < x_min:
                x_min = point.x
            if point.x > x_max:
                x_max = point.x
            if point.y < y_min:
                y_min = point.y
            if point.y > y_max:
                y_max = point.y

        # check is robot in trigger region
        if self.position.x >= x_min and \
                self.position.x <= x_max and \
                self.position.y >= y_min and \
                self.position.y <= y_max:
            return True
        else:
            return False

    def send_goal(self, goal):
        self.move_base_client.send_goal(goal)

    def flow(self):
        if self.nav_stage == NavStage.IDLE:
            pass  # do nothing
        if self.nav_stage == NavStage.TO_SCAN:
            self.send_goal(self.scan_goal)
            self.nav_stage = NavStage.ENROUTE_TO_SCAN
        if self.nav_stage == NavStage.ENROUTE_TO_SCAN:
            if self.is_in_region(self.scan_region):
                self.nav_stage = NavStage.SCAN_CODE
                print("start QR code scanning")
        if self.nav_stage == NavStage.SCAN_CODE:
            self._scan_qr_code()
        if self.nav_stage == NavStage.TO_DESTINATION:
            self.send_goal(self.end_goal)
            self.nav_stage = NavStage.IDLE

    def publish_visualizations(self):
        self.pub_scan_region.publish(self.scan_region)

    def run(self):
        sleep(0.2)
        print()
        print()
        print()
        print()
        print("================================================")
        print("=  Commander is ready, wating start signal...  =")
        print("================================================")
        print()
        print()
        print()
        print()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._get_robot_pose()
            self.flow()
            self.publish_visualizations()
            rate.sleep()


def main():
    node = UcarCommander()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n')
        print('操作已取消')
        exit(0)
