#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import actionlib
from enum import Enum
from std_srvs.srv import SetBool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
from qr_msgs.srv import qr as QRsrv


class NavStage(Enum):
    IDLE = 0
    TO_SCAN = 1
    ENROUTE_TO_SCAN = 2
    SCAN_CODE = 3
    TO_DESTINATION = 4
    ENROUTE_TO_DESTINATION = 4


class UcarCommander(object):

    def __init__(self) -> None:
        # ROS Infastructure
        rospy.init_node("ucar_commander_node")
        self.tf_listener = tf.TransformListener()
        self.srv_nav_start = rospy.Service(
            'nav_start', SetBool, self.nav_start_srv_handler)
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.pub_scan_region = rospy.Publisher(
            'scan_region', PolygonStamped, queue_size=1)

        # vars
        self.global_frame_id = "map"
        self.robot_frame_id = "base_link"

        self.is_nav_start = False
        self.nav_stage = NavStage.IDLE
        self.position = Point32()

        self.scan_goal = MoveBaseGoal()

        self.scan_goal.target_pose.header.frame_id = self.global_frame_id
        self.scan_goal.target_pose.pose.position.x = 2.771
        self.scan_goal.target_pose.pose.position.y = -3.235
        self.scan_goal.target_pose.pose.position.z = 0
        self.scan_goal.target_pose.pose.orientation.x = 0
        self.scan_goal.target_pose.pose.orientation.y = 0
        self.scan_goal.target_pose.pose.orientation.z = 1
        self.scan_goal.target_pose.pose.orientation.w = 0

        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()

        pt1.x = 2.242
        pt1.y = -3.306
        pt2.x = 2.242
        pt2.y = -2.871
        pt3.x = 3.318
        pt3.y = -2.871
        pt4.x = 3.318
        pt4.y = -3.306

        self.scan_region = PolygonStamped()
        self.scan_region.header.frame_id = self.global_frame_id
        self.scan_region.polygon.points.append(pt1)
        self.scan_region.polygon.points.append(pt2)
        self.scan_region.polygon.points.append(pt3)
        self.scan_region.polygon.points.append(pt4)

    def nav_start_srv_handler(self, req):
        if not self.is_nav_start:
            if req.data:
                print("Commander Start!!")
                self.is_nav_start = True
                self.nav_stage = NavStage.TO_SCAN
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
            print("service call to scan qr code failed: " + str(ex))
            return -1

    def _scan_qr_code(self):
        id = self.code_scan_srv_client()
        if id == -1:
            pass
        elif id >= 0 and id <= 2:
            print("Got QR code: " + str(id))
            # 根据扫码结果设置终点目标
            self.nav_stage = NavStage.TO_DESTINATION

    def send_goal(self, goal):
        self.move_base_client.send_goal(goal)

    def _get_robot_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            self.position.x = trans[0]
            self.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup failed")

    def publish_visuliaztion(self):
        self.pub_scan_region.publish(self.scan_region)

    def is_in_region(self, region):
        # check if robot is in a region
        x_min = region.polygon.points[0].x
        x_max = region.polygon.points[0].x
        y_min = region.polygon.points[0].y
        y_max = region.polygon.points[0].y

        # find min and max x, y of the region
        for point in region.polygon.points:
            if point.x < x_min:
                x_min = point.x
            if point.x > x_max:
                x_max = point.x
            if point.y < y_min:
                y_min = point.y
            if point.y > y_max:
                y_max = point.y

        # check if robot position is within that range
        if self.position.x >= x_min and \
                self.position.x <= x_max and \
                self.position.y >= y_min and \
                self.position.y <= y_max:
            return True
        else:
            return False

    # FLOW CONTROL

    def flow(self):
        if self.nav_stage == NavStage.IDLE:
            pass  # do nothing

        if self.nav_stage == NavStage.TO_SCAN:
            self.send_goal(self.scan_goal)
            self.nav_stage = NavStage.ENROUTE_TO_SCAN

        if self.nav_stage == NavStage.ENROUTE_TO_SCAN:
            if self.is_in_region(self.scan_region):
                self.nav_stage = NavStage.SCAN_CODE
                print("scan QR code")

        if self.nav_stage == NavStage.SCAN_CODE:
            self._scan_qr_code()

        if self.nav_stage == NavStage.TO_DESTINATION:
            print("go to destination")
            # 发送前往终点的坐标
            self.nav_stage = NavStage.ENROUTE_TO_DESTINATION

        if self.nav_stage == NavStage.ENROUTE_TO_DESTINATION:
            pass  # do noting

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._get_robot_position()
            self.publish_visuliaztion()
            self.flow()
            rate.sleep()


def main():
    node = UcarCommander()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
