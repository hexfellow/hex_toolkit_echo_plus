#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-02
################################################################

import numpy as np

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface
from utility import PdCtrl
from utility import DistYawCtrl

import hex_utils
from hex_utils import ObsUtilWork
from hex_utils import HexCartVel, HexCartPose, HexCartState


class PidTrace:

    def __init__(self):
        ### data interface
        self.__data_interface = DataInterface("pid_trace")

        ### parameter
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__limit_param = self.__data_interface.get_limit_param()
        self.__obs_param = self.__data_interface.get_obs_param()
        self.__trace_param = self.__data_interface.get_trace_param()

        ### utility
        self.__obs_util = ObsUtilWork(
            dt=1.0 / self.__rate_param['ros'],
            vel_limit=self.__limit_param['vel'],
            acc_limit=self.__limit_param['acc'],
        )
        self.__pd_ctrl = PdCtrl(trace_param=self.__trace_param)
        self.__dist_yaw_ctrl = DistYawCtrl(trace_param=self.__trace_param)

        ### variables
        # current target
        self.__cur_tar = None
        # pid result
        self.__x_arr = []
        # control message
        self.__ctrl_msg = HexCartVel()

    def __preprocess(self, cur_state: HexCartState, cur_tar: HexCartPose):
        trans_odom2cur = cur_state.pose().get_trans()
        trans_odom2tar = cur_tar.get_trans()

        # cur
        lin_cur = cur_state.vel().linear()
        ang_cur = cur_state.vel().angular()
        v_cur = np.array([lin_cur[0], ang_cur[2]])

        # ref
        trans_cur2tar = hex_utils.trans_inv(trans_odom2cur) @ trans_odom2tar
        pos_err, quat_ref = hex_utils.trans2part(trans_cur2tar)
        yaw_err = hex_utils.quat2yaw(quat_ref)
        if np.linalg.norm(pos_err) < 0.05:
            yaw_ref = yaw_err
            pos_ref = np.zeros(2)
        else:
            yaw_ref, pos_ref = self.__dist_yaw_ctrl(
                yaw_err=yaw_err,
                pos_err=pos_err,
                v_cur=lin_cur[0],
            )
        p_ref = np.array([pos_ref[0], yaw_ref])

        return p_ref, v_cur

    def run(self):
        while self.__data_interface.ok():
            # update odom
            sensor_odom = None
            while self.__data_interface.has_chassis_odom():
                sensor_odom = self.__data_interface.get_chassis_odom()
            if sensor_odom is not None:
                if self.__cur_tar is None:
                    self.__cur_tar = sensor_odom.get_pose()
                if self.__obs_util.is_ready():
                    self.__obs_util.update(
                        sensor_odom,
                        self.__obs_param['weights'],
                    )
                else:
                    self.__obs_util.set_state(sensor_odom)

            if self.__obs_util.is_ready():
                # update target
                tar_cart = None
                while self.__data_interface.has_target_pose():
                    tar_cart = self.__data_interface.get_target_pose()
                if tar_cart is not None:
                    self.__cur_tar = tar_cart

                # get x
                cur_state = self.__obs_util.get_state()
                p_ref, v_cur = self.__preprocess(cur_state, self.__cur_tar)

                # get u
                acc = self.__pd_ctrl(
                    p_cur=np.zeros(2),
                    p_ref=p_ref,
                    v_cur=v_cur,
                    v_ref=np.zeros(2),
                )

                # get control message
                self.__obs_util.predict(
                    acc_lin=np.array([acc[0], 0.0, 0.0]),
                    acc_ang=np.array([0.0, 0.0, acc[1]]),
                )
                cur_state = self.__obs_util.get_state()
                self.__ctrl_msg = cur_state.vel()

                # pub ctrl
                self.__data_interface.pub_unsafe_ctrl(self.__ctrl_msg)
                self.__data_interface.pub_vel_ctrl(self.__ctrl_msg)

            # sleep
            self.__data_interface.sleep()


def main():
    pid_trace = PidTrace()
    pid_trace.run()


if __name__ == '__main__':
    main()
