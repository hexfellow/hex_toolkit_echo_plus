#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import copy
import numpy as np

from .math_util import MathUtil
from .hex_struct import HexCartState


class ObsUtil:

    def __init__(
        self,
        rate_param: dict,
        obs_param: dict,
    ):
        ### parameters
        self.__rate_param = copy.deepcopy(rate_param)
        self.__obs_param = copy.deepcopy(obs_param)

        ### variables
        self.__ready = False
        self.__obs_state = None

    def is_ready(self) -> bool:
        return self.__ready

    def set_state(self, state: HexCartState):
        self.__obs_state = state
        self.__ready = True

    def update(self, state: HexCartState):
        obs_pos = self.__obs_state.get_pos()
        obs_quat = self.__obs_state.get_quat()
        obs_vel = self.__obs_state.get_vel()
        obs_omega = self.__obs_state.get_omega()

        cur_pos = state.get_pos()
        cur_quat = state.get_quat()
        cur_vel = state.get_vel()
        cur_omega = state.get_omega()

        obs_weight = self.__obs_param["weights"]
        cur_weight = 1.0 - obs_weight
        new_pos = obs_weight * obs_pos + cur_weight * cur_pos
        new_quat = MathUtil.quat_slerp(obs_quat, cur_quat, cur_weight)
        new_vel = obs_weight * obs_vel + cur_weight * cur_vel
        new_omega = obs_weight * obs_omega + cur_weight * cur_omega

        self.__obs_state.set_pos(new_pos)
        self.__obs_state.set_quat(new_quat)
        self.__obs_state.set_vel(new_vel)
        self.__obs_state.set_omega(new_omega)

    def get_state(self) -> HexCartState:
        return self.__obs_state

    def predict(
        self,
        tar_vel: np.ndarray,
        tar_omega: np.ndarray,
    ):
        pos = self.__obs_state.get_pos()
        quat = self.__obs_state.get_quat()
        vel = self.__obs_state.get_vel()
        omega = self.__obs_state.get_omega()

        avg_vel = (vel + tar_vel) / 2.0
        new_pos = pos + avg_vel / self.__rate_param["ros"]

        avg_omega = (omega + tar_omega) / 2.0
        delta_angle = avg_omega / self.__rate_param["ros"]
        if np.linalg.norm(delta_angle) > 1e-6:
            new_quat = copy.deepcopy(quat)
        else:
            angle_axis = delta_angle / np.linalg.norm(delta_angle)
            cos_factor = np.cos(delta_angle[0] / 2.0)
            sin_factor = np.sin(delta_angle[0] / 2.0)
            delta_quat = np.array([
                cos_factor,
                sin_factor * angle_axis[0],
                sin_factor * angle_axis[1],
                sin_factor * angle_axis[2],
            ])
            new_quat = MathUtil.quat_mul(delta_quat, quat)

        self.__obs_state.set_pos(new_pos)
        self.__obs_state.set_quat(new_quat)
        self.__obs_state.set_vel(tar_vel)
        self.__obs_state.set_omega(tar_omega)
