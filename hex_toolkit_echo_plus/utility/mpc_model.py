#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-20
################################################################

import casadi as ca
from acados_template import AcadosModel


class ChassisModel(object):

    def __init__(
        self,
        model_param: dict,
        limit_param: dict,
        mpc_param: dict,
    ):
        ############################
        # model
        ############################
        model = AcadosModel()

        ### model states ###
        p_x = ca.SX.sym('p_x')
        p_y = ca.SX.sym('p_y')
        yaw = ca.SX.sym('yaw')
        v_l = ca.SX.sym('v_l')
        v_r = ca.SX.sym('v_r')
        model.x = states = ca.vcat([p_x, p_y, yaw, v_l, v_r])

        ### control inputs ###
        a_l = ca.SX.sym('a_l')
        a_r = ca.SX.sym('a_r')
        model.u = controls = ca.vcat([a_l, a_r])

        ### dynamic function ###
        # explicit dynamic function
        # `rhs` is the right hand side of the differential equation
        vel = (v_l + v_r) * 0.5
        omega = (v_r - v_l) / model_param["track_width"]
        rhs = [
            vel * ca.cos(yaw),
            vel * ca.sin(yaw),
            omega,
            a_l,
            a_r,
        ]
        f_expl = ca.Function(
            'f',
            [states, controls],
            [ca.vcat(rhs)],
            ['state', 'control'],
            ['rhs'],
        )
        # explicit model setting
        model.f_expl_expr = f_expl(states, controls)
        # other settings
        acc = (a_l + a_r) * 0.5
        alpha = (a_r - a_l) / model_param["track_width"]
        model.con_h_expr = ca.vcat([vel, omega, acc, alpha])

        ### cost function ###
        model.cost_y_expr = ca.vcat([model.x, model.u])
        model.cost_y_expr_e = ca.vcat([model.x[:3]])

        ### other settings ###
        model.p = ca.vcat([])
        model.name = "echo_plus"

        ############################
        # constraint
        ############################
        constraint = ca.types.SimpleNamespace()

        # constraints of states
        constraint.vl_min = mpc_param["vel"][0, 0]
        constraint.vl_max = mpc_param["vel"][0, 1]
        constraint.vr_min = mpc_param["vel"][1, 0]
        constraint.vr_max = mpc_param["vel"][1, 1]

        # constraints of controls
        constraint.al_min = mpc_param["ctrl"][0, 0]
        constraint.al_max = mpc_param["ctrl"][0, 1]
        constraint.ar_min = mpc_param["ctrl"][1, 0]
        constraint.ar_max = mpc_param["ctrl"][1, 1]

        # constraints of others
        constraint.vel_min = limit_param["vel"][0, 0]
        constraint.vel_max = limit_param["vel"][0, 1]
        constraint.omega_min = limit_param["vel"][2, 0]
        constraint.omega_max = limit_param["vel"][2, 1]
        constraint.acc_min = limit_param["acc"][0, 0]
        constraint.acc_max = limit_param["acc"][0, 1]
        constraint.alpha_min = limit_param["acc"][2, 0]
        constraint.alpha_max = limit_param["acc"][2, 1]

        self.model = model
        self.constraint = constraint
