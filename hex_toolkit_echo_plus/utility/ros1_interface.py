#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import numpy as np

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from .interface_base import InterfaceBase
from hex_utils import HexCartVel, HexCartPose, HexCartState


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rospy.init_node(self._name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 300.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### pamameter
        # rate
        self._rate_param.update({
            "odom": rospy.get_param('~rate_odom', 100.0),
        })
        # model
        self._model_param.update({
            "path":
            rospy.get_param('~model_path', "unknown"),
            "base":
            rospy.get_param('~model_base', "unknown"),
            "odom":
            rospy.get_param('~model_odom', "unknown"),
        })
        # obs
        self._obs_param.update({
            "weights":
            np.array(rospy.get_param('~obs_weights', [1.0])),
        })
        # limit
        self._limit_param.update({
            "vel":
            np.array(rospy.get_param('~limit_vel', [0.0])),
            "acc":
            np.array(rospy.get_param('~limit_acc', [0.0])),
        })
        # trace
        self._trace_param.update({
            "pid":
            np.array(
                self._str_to_list(rospy.get_param('~trace_pid',
                                                  ["[1.0, 1.0]"]))),
            "dt":
            1.0 / self._rate_param["ros"],
            "err_limit":
            np.array(rospy.get_param('~trace_err_limit', [0.0])),
            "stanley":
            rospy.get_param('~trace_stanley', 1.0),
        })

        ### publisher
        self.__unsafe_ctrl_pub = rospy.Publisher(
            'unsafe_ctrl',
            Twist,
            queue_size=10,
        )
        self.__vel_ctrl_pub = rospy.Publisher(
            'vel_ctrl',
            TwistStamped,
            queue_size=10,
        )

        ### subscriber
        self.__chassis_odom_sub = rospy.Subscriber(
            'chassis_odom',
            Odometry,
            self.__chassis_odom_callback,
        )
        self.__target_pose_sub = rospy.Subscriber(
            'target_pose',
            PoseStamped,
            self.__target_pose_callback,
        )
        self.__chassis_odom_sub
        self.__target_pose_sub

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    def pub_unsafe_ctrl(self, out: HexCartVel):
        vel = out.get_linear()
        omega = out.get_angular()

        msg = Twist()
        msg.linear.x = vel[0]
        msg.linear.y = vel[1]
        msg.linear.z = vel[2]
        msg.angular.x = omega[0]
        msg.angular.y = omega[1]
        msg.angular.z = omega[2]

        self.__unsafe_ctrl_pub.publish(msg)

    def pub_vel_ctrl(self, out: HexCartVel):
        vel = out.get_linear()
        omega = out.get_angular()

        msg = TwistStamped()
        msg.twist.linear.x = vel[0]
        msg.twist.linear.y = vel[1]
        msg.twist.linear.z = vel[2]
        msg.twist.angular.x = omega[0]
        msg.twist.angular.y = omega[1]
        msg.twist.angular.z = omega[2]

        msg.header.frame_id = self._model_param["base"]
        msg.header.stamp = rospy.Time.now()
        self.__vel_ctrl_pub.publish(msg)

    def __chassis_odom_callback(self, msg: Odometry):
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])
        vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        omega = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])
        odom = HexCartState(
            pose=HexCartPose(pos=pos, quat=quat),
            vel=HexCartVel(linear=vel, angular=omega),
        )
        self._chassis_odom_queue.put(odom)

    def __target_pose_callback(self, msg: PoseStamped):
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        quat = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])
        pose = HexCartPose(pos=pos, quat=quat)
        self._target_pose_queue.put(pose)
