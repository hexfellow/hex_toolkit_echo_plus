#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np

import rclpy
import rclpy.node
import threading
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
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 300.0)
        self._rate_param["ros"] = self.__node.get_parameter('rate_ros').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### pamameter
        # declare parameters
        self.__node.declare_parameter('rate_odom', 100.0)
        self.__node.declare_parameter('model_path', "unknown")
        self.__node.declare_parameter('model_base', "unknown")
        self.__node.declare_parameter('model_odom', "unknown")
        self.__node.declare_parameter('limit_vel', [0.0])
        self.__node.declare_parameter('limit_acc', [0.0])
        self.__node.declare_parameter('obs_weights', [0.5])
        self.__node.declare_parameter('trace_pid', ["[1.0, 1.0]"])
        self.__node.declare_parameter('trace_err_limit', [0.0])
        self.__node.declare_parameter('trace_switch_dist', 0.1)
        # rate
        self._rate_param.update({
            "odom":
            self.__node.get_parameter('rate_odom').value,
        })
        # model
        self._model_param.update({
            "path":
            self.__node.get_parameter('model_path').value,
            "base":
            self.__node.get_parameter('model_base').value,
            "odom":
            self.__node.get_parameter('model_odom').value,
        })
        # limit
        self._limit_param.update({
            "vel":
            np.array(self.__node.get_parameter('limit_vel').value),
            "acc":
            np.array(self.__node.get_parameter('limit_acc').value),
        })
        # obs
        self._obs_param.update({
            "weights":
            np.array(self.__node.get_parameter('obs_weights').value),
        })
        # trace
        self._trace_param.update({
            "pid":
            np.array(
                self._str_to_list(
                    self.__node.get_parameter('trace_pid').value)),
            "dt":
            1.0 / self._rate_param["ros"],
            "err_limit":
            np.array(self.__node.get_parameter('trace_err_limit').value),
            "switch_dist":
            self.__node.get_parameter('trace_switch_dist').value,
        })

        ### publisher
        self.__unsafe_ctrl_pub = self.__node.create_publisher(
            Twist,
            'unsafe_ctrl',
            10,
        )
        self.__vel_ctrl_pub = self.__node.create_publisher(
            TwistStamped,
            'vel_ctrl',
            10,
        )

        ### subscriber
        self.__chassis_odom_sub = self.__node.create_subscription(
            Odometry,
            'chassis_odom',
            self.__chassis_odom_callback,
            10,
        )
        self.__target_pose_sub = self.__node.create_subscription(
            PoseStamped,
            'target_pose',
            self.__target_pose_callback,
            10,
        )
        self.__chassis_odom_sub
        self.__target_pose_sub

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

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
        msg.header.stamp = self.__node.get_clock().now().to_msg()
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
