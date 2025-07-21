#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-02
################################################################

import numpy as np

import hex_utils


class PdCtrl:

    def __init__(
        self,
        trace_param: dict,
    ):
        self.__kp = trace_param["pid"][0]
        self.__kd = trace_param["pid"][2]
        self.__dt = trace_param["dt"]
        self.__err_limit = trace_param["err_limit"]

    def __call__(self, p_cur, p_ref, v_cur, v_ref=np.zeros(1)):
        p_err = p_ref - p_cur
        p_err[1] = hex_utils.angle_norm(p_err[1])
        p_err = np.clip(p_err, -self.__err_limit, self.__err_limit)
        v_err = v_ref - v_cur
        acc = self.__kp * p_err + self.__kd * v_err
        return acc


# class StanleyCtrl:

#     def __init__(
#         self,
#         trace_param: dict,
#     ):
#         # yaw_ref = yaw_err + atan2(k_stanley * dist, v_cur)
#         self.__k_stanley = trace_param["stanley"]

#     def __call__(self, yaw_err, pos_err, v_cur):
#         # cos_yaw * (y - y_0) = sin_yaw * (x - x_0)
#         # sin_yaw * x - cos_yaw * y - sin_yaw * x_0 + cos_yaw * y_0 = 0
#         # A * x + B * y + C = 0
#         sin_yaw = np.sin(yaw_err)
#         cos_yaw = np.cos(yaw_err)
#         coeff_c = -sin_yaw * pos_err[0] + cos_yaw * pos_err[1]
#         dis_val = coeff_c
#         yaw_ref = yaw_err + np.arctan2(
#             self.__k_stanley * dis_val,
#             v_cur,
#         )
#         return yaw_ref


class DistYawCtrl:

    def __init__(
        self,
        trace_param: dict,
    ):
        self.__pi_6 = np.pi / 6

        self.__switch_dist = trace_param["switch_dist"]

    def __dist_yaw(self, pos_err):
        return np.arctan2(pos_err[1], pos_err[0])

    def __call__(self, yaw_err, pos_err, v_cur):
        yaw_ref = yaw_err
        pos_ref = pos_err
        if np.linalg.norm(pos_err) > self.__switch_dist:
            yaw_ref = self.__dist_yaw(pos_ref)
        else:
            yaw_ref = yaw_err

        yaw_norm = np.fabs(yaw_ref)
        if yaw_norm > self.__pi_6:
            pos_ref = np.zeros(2)

        return yaw_ref, pos_ref
