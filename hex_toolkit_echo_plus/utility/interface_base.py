#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import json
import queue
import typing
from abc import ABC, abstractmethod
from hex_utils import HexCartVel, HexCartPose, HexCartState


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}
        self._model_param = {}
        self._limit_param = {}
        self._obs_param = {}
        self._trace_param = {}

        ### rx msg queues
        self._chassis_odom_queue = queue.Queue()
        self._target_pose_queue = queue.Queue()

        ### name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        self.shutdown()

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    ####################
    ### parameters
    ####################
    def _str_to_list(self, list_str) -> list:
        result = []
        for s in list_str:
            l = json.loads(s)
            result.append(l)
        return result

    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_model_param(self) -> dict:
        return self._model_param

    def get_limit_param(self) -> dict:
        return self._limit_param

    def get_obs_param(self) -> dict:
        return self._obs_param

    def get_trace_param(self) -> dict:
        return self._trace_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_unsafe_ctrl(self, out: HexCartVel):
        raise NotImplementedError("InterfaceBase.pub_unsafe_ctrl")

    @abstractmethod
    def pub_vel_ctrl(self, out: HexCartVel):
        raise NotImplementedError("InterfaceBase.pub_vel_ctrl")

    ####################
    ### subscribers
    ####################
    # chassis odom
    def has_chassis_odom(self) -> bool:
        return not self._chassis_odom_queue.empty()

    def clear_chassis_odom(self):
        self._chassis_odom_queue.queue.clear()

    def get_chassis_odom(self) -> typing.Optional[HexCartState]:
        try:
            return self._chassis_odom_queue.get_nowait()
        except queue.Empty:
            return None

    # target pose
    def has_target_pose(self) -> bool:
        return not self._target_pose_queue.empty()

    def clear_target_pose(self):
        self._target_pose_queue.queue.clear()

    def get_target_pose(self) -> typing.Optional[HexCartPose]:
        try:
            return self._target_pose_queue.get_nowait()
        except queue.Empty:
            return None
