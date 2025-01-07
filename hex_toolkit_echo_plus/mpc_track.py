#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import time
import numpy as np

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface
from utility import ObsUtil
from utility import MpcUtil
from utility import MathUtil
from utility import HexCartVel, HexCartPose, HexCartState


class MpcTrack:

    def __init__(self):
        ### data interface
        self.__data_interface = DataInterface("mpc_track")

        ### parameter
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__limit_param = self.__data_interface.get_limit_param()
        self.__obs_param = self.__data_interface.get_obs_param()
        self.__mpc_param = self.__data_interface.get_mpc_param()

        ### utility
        self.__obs_util = ObsUtil(
            rate_param=self.__rate_param,
            obs_param=self.__obs_param,
        )
        self.__mpc_util = MpcUtil(
            self.__rate_param,
            self.__model_param,
            self.__limit_param,
            self.__mpc_param,
        )

        ### variables
        # update interval
        self.__update_interval = int(self.__rate_param["odom"] /
                                     self.__rate_param["mpc"])
        # current target
        self.__cur_tar = None
        # mpc result
        self.__x_arr = []
        # control message
        self.__ctrl_msg = HexCartVel()

    def __cart2motor(self, cart_vel: HexCartVel):
        cart_np = np.array((
            cart_vel.get_vel()[0],
            cart_vel.get_omega()[2],
        ))
        motor_np = self.__model_param["cart2motor"] @ cart_np
        return motor_np

    def __motor2cart(self, motor_vel: np.ndarray):
        cart_vel = self.__model_param["motor2cart"] @ motor_vel
        return HexCartVel(
            vel=np.array([cart_vel[0], 0.0, 0.0]),
            omega=np.array([0.0, 0.0, cart_vel[1]]),
        )

    def __preprocess(self, cur_state: HexCartState, cur_tar: HexCartPose):
        cur_pos = cur_state.get_pos()
        cur_quat = cur_state.get_quat()
        trans_odom2cur = MathUtil.part2trans(cur_pos, cur_quat)
        tar_pos = cur_tar.get_pos()
        tar_quat = cur_tar.get_quat()
        trans_odom2tar = MathUtil.part2trans(tar_pos, tar_quat)

        # x_init
        cur_motor = self.__cart2motor(cur_state.get_cart_vel())
        x_init = np.concatenate((
            np.zeros(3),
            cur_motor,
        ))

        # y_ref
        trans_cur2tar = MathUtil.trans_inv(trans_odom2cur) @ trans_odom2tar
        pos_ref, quat_ref = MathUtil.trans2part(trans_cur2tar)
        yaw_ref = MathUtil.quat2yaw(quat_ref)
        y_ref = np.array([pos_ref[0], pos_ref[1], yaw_ref])

        return x_init, y_ref

    def __postprocess(self, cur_x: np.ndarray):
        return self.__motor2cart(cur_x[3:])

    def __mpc_solve(self, cur_state: HexCartState):
        x_init, y_ref = self.__preprocess(cur_state, self.__cur_tar)
        _, self.__x_arr = self.__mpc_util.solve(x_init, y_ref)
        self.__x_arr.pop(0)

    def run(self):
        update_count = 0
        while self.__data_interface.ok():
            # update odom
            sensor_odom = None
            while self.__data_interface.has_chassis_odom():
                sensor_odom = self.__data_interface.get_chassis_odom()
            if sensor_odom is not None:
                if self.__cur_tar is None:
                    self.__cur_tar = HexCartPose(
                        pos=sensor_odom.get_pos(),
                        quat=sensor_odom.get_quat(),
                    )
                    # init mpc state
                    x_init, y_ref = self.__preprocess(sensor_odom,
                                                      self.__cur_tar)
                    self.__mpc_util.set_state(x_init, y_ref)
                if self.__obs_util.is_ready():
                    if update_count == 0:
                        self.__obs_util.update(sensor_odom)
                    update_count = (update_count + 1) % self.__update_interval
                else:
                    self.__obs_util.set_state(sensor_odom)
                # mpc solve
                start = time.perf_counter()
                self.__mpc_solve(sensor_odom)
                print(
                    f"mpc cost time: {(time.perf_counter() - start) * 1e3} ms")

            if self.__obs_util.is_ready():
                # update target
                tar_cart = None
                while self.__data_interface.has_target_pose():
                    tar_cart = self.__data_interface.get_target_pose()
                if tar_cart is not None:
                    self.__cur_tar = tar_cart

                # get control message
                cur_x = self.__x_arr.pop(0)
                self.__ctrl_msg = self.__postprocess(cur_x)

                # pub ctrl
                self.__data_interface.pub_unsafe_ctrl(self.__ctrl_msg)
                self.__data_interface.pub_vel_ctrl(self.__ctrl_msg)

            # sleep
            self.__data_interface.sleep()


def main():
    mpc_track = MpcTrack()
    mpc_track.run()


if __name__ == '__main__':
    main()
