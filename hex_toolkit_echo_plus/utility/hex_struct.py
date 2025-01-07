#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import copy
import numpy as np


class HexCartVel:
    # vel: [x, y, z]
    # omega: [x, y, z]

    def __init__(
            self,
            vel: np.ndarray = np.array([0.0, 0.0, 0.0]),
            omega: np.ndarray = np.array([0.0, 0.0, 0.0]),
    ):
        self.__vel = copy.deepcopy(vel)
        self.__omega = copy.deepcopy(omega)

    def __repr__(self):
        print_str = f"vel: {self.__vel}\n"
        print_str += f"omega: {self.__omega}"
        return print_str

    def set_vel(self, vel: np.ndarray):
        self.__vel = copy.deepcopy(vel)

    def set_omega(self, omega: np.ndarray):
        self.__omega = copy.deepcopy(omega)

    def get_vel(self):
        return self.__vel

    def get_omega(self):
        return self.__omega


class HexCartPose:
    # pos: [x, y, z]
    # quat: [w, x, y, z]

    def __init__(
            self,
            pos: np.ndarray = np.array([0.0, 0.0, 0.0]),
            quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0]),
    ):
        self.__pos = copy.deepcopy(pos)
        self.__quat = copy.deepcopy(quat)

    def __repr__(self):
        print_str = f"pos: {self.__pos}\n"
        print_str += f"quat: {self.__quat}"
        return print_str

    def set_pos(self, pos: np.ndarray):
        self.__pos = copy.deepcopy(pos)

    def set_quat(self, quat: np.ndarray):
        self.__quat = copy.deepcopy(quat)

    def get_pos(self):
        return self.__pos

    def get_quat(self):
        return self.__quat


class HexCartState:
    # pos: [x, y, z]
    # quat: [w, x, y, z]
    # vel: [x, y, z]
    # omega: [x, y, z]

    def __init__(
            self,
            pos: np.ndarray = np.array([0.0, 0.0, 0.0]),
            quat: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0]),
            vel: np.ndarray = np.array([0.0, 0.0, 0.0]),
            omega: np.ndarray = np.array([0.0, 0.0, 0.0]),
    ):
        self.__cart_vel = HexCartVel(vel, omega)
        self.__cart_pose = HexCartPose(pos, quat)

    def __repr__(self):
        print_str = f"{self.__cart_pose}\n"
        print_str += f"{self.__cart_vel}"
        return print_str

    def set_pos(self, pos: np.ndarray):
        self.__cart_pose.set_pos(pos)

    def set_quat(self, quat: np.ndarray):
        self.__cart_pose.set_quat(quat)

    def set_vel(self, vel: np.ndarray):
        self.__cart_vel.set_vel(vel)

    def set_omega(self, omega: np.ndarray):
        self.__cart_vel.set_omega(omega)

    def set_cart_pose(self, cart_pose: HexCartPose):
        self.__cart_pose = copy.deepcopy(cart_pose)

    def set_cart_vel(self, cart_vel: HexCartVel):
        self.__cart_vel = copy.deepcopy(cart_vel)

    def get_pos(self):
        return self.__cart_pose.get_pos()

    def get_quat(self):
        return self.__cart_pose.get_quat()

    def get_vel(self):
        return self.__cart_vel.get_vel()

    def get_omega(self):
        return self.__cart_vel.get_omega()

    def get_cart_pose(self):
        return self.__cart_pose

    def get_cart_vel(self):
        return self.__cart_vel
