#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import os
import sys
import copy
import errno
import shutil
import numpy as np
from typing import Tuple
from acados_template import AcadosOcp, AcadosOcpSolver

from .mpc_model import ChassisModel


class MpcUtil:

    def __init__(
        self,
        rate_param: dict,
        model_param: dict,
        limit_param: dict,
        mpc_param: dict,
    ):
        ### parameters
        self.__rate_param = copy.deepcopy(rate_param)
        self.__model_param = copy.deepcopy(model_param)
        self.__limit_param = copy.deepcopy(limit_param)
        self.__mpc_param = copy.deepcopy(mpc_param)

        # prepare directory
        mpc_path = f"{self.__mpc_param['root']}/temp"
        self.__safe_mkdir_recursive(mpc_path)
        self.__safe_mkdir_recursive(f"{mpc_path}/acados_models")
        os.chdir(mpc_path)
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)

        # model
        self.__model = ChassisModel(
            self.__model_param,
            self.__limit_param,
            self.__mpc_param,
        )

        # states and control dimensions
        self.__dim_x = self.__model.model.x.size()[0]
        self.__dim_u = self.__model.model.u.size()[0]
        self.__dim_p = self.__model.model.p.size()[0]
        self.__horizon = self.__mpc_param['window'] / self.__rate_param['ros']
        self.__num_window = self.__mpc_param['window']

        # create ocp object
        self.__ocp = AcadosOcp()
        self.__ocp.acados_include_path = f"{acados_source_path}/include"
        self.__ocp.acados_lib_path = f"{acados_source_path}/lib"
        self.__ocp.model = self.__model.model
        self.__ocp.dims.N = self.__num_window
        self.__ocp.solver_options.tf = self.__horizon

        ### initialize ocp parameters
        self.__ocp.dims.np = self.__dim_p
        self.__ocp.parameter_values = np.zeros(self.__dim_p)

        # cost initialization
        dim_y = self.__dim_x + self.__dim_u
        # cost middle
        Q = np.diag(
            np.concatenate((
                self.__mpc_param["mid_wt"][0] * np.ones(2),
                self.__mpc_param["mid_wt"][1] * np.ones(1),
                self.__mpc_param["mid_wt"][2] * np.ones(2),
            )))
        R = np.diag(self.__mpc_param["mid_wt"][3] * np.ones(self.__dim_u))
        self.__ocp.cost.cost_type = "NONLINEAR_LS"
        self.__ocp.cost.W = np.block([
            [Q, np.zeros((dim_y - self.__dim_u, self.__dim_u))],
            [np.zeros((self.__dim_u, dim_y - self.__dim_u)), R],
        ])
        # cost end
        Q_e = np.diag(
            np.concatenate((
                self.__mpc_param["end_wt"][0] * np.ones(2),
                self.__mpc_param["end_wt"][1] * np.ones(1),
            )))
        self.__ocp.cost.cost_type_e = "NONLINEAR_LS"
        self.__ocp.cost.W_e = Q_e

        # constraints
        # hard constraints
        # x
        self.__ocp.constraints.lbx = np.array(
            [self.__model.constraint.vl_min, self.__model.constraint.vr_min])
        self.__ocp.constraints.ubx = np.array(
            [self.__model.constraint.vl_max, self.__model.constraint.vr_max])
        self.__ocp.constraints.idxbx = np.array([3, 4])
        # u
        self.__ocp.constraints.lbu = np.array(
            [self.__model.constraint.al_min, self.__model.constraint.ar_min])
        self.__ocp.constraints.ubu = np.array(
            [self.__model.constraint.al_max, self.__model.constraint.ar_max])
        self.__ocp.constraints.idxbu = np.array([0, 1])
        # h
        self.__ocp.constraints.lh = np.array([
            self.__model.constraint.vel_min, self.__model.constraint.omega_min,
            self.__model.constraint.acc_min, self.__model.constraint.alpha_min
        ])
        self.__ocp.constraints.uh = np.array([
            self.__model.constraint.vel_max, self.__model.constraint.omega_max,
            self.__model.constraint.acc_max, self.__model.constraint.alpha_max
        ])

        # initial state
        x_ref = np.zeros(self.__dim_x)
        u_ref = np.zeros(self.__dim_u)
        y_ref = np.concatenate((x_ref, u_ref))
        y_ref_e = np.zeros(self.__dim_x - 2)
        self.__ocp.constraints.x0 = x_ref
        self.__ocp.cost.yref = y_ref
        self.__ocp.cost.yref_e = y_ref_e

        # solver options
        self.__ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        self.__ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        self.__ocp.solver_options.integrator_type = "ERK"
        self.__ocp.solver_options.print_level = 0
        self.__ocp.solver_options.nlp_solver_type = "SQP_RTI"

        # create acados solver and integrator
        self.__json_path = f"{mpc_path}/{self.__model.model.name}_acados_ocp.json"
        self.__solver = AcadosOcpSolver(self.__ocp, json_file=self.__json_path)

    def __safe_mkdir_recursive(self, dir, overwrite=False):
        if not os.path.exists(dir):
            try:
                os.makedirs(dir)
            except OSError as exc:
                if exc.errno == errno.EEXIST and os.path.isdir(dir):
                    pass
                else:
                    raise
        else:
            if overwrite:
                try:
                    shutil.rmtree(dir)
                except:
                    print('Error while removing directory {}'.format(dir))

    def set_state(
        self,
        x_init: np.ndarray,
        y_ref: np.ndarray,
    ):
        x0 = copy.deepcopy(x_init)
        yref = np.concatenate((
            y_ref,
            np.zeros(2),
            np.zeros(self.__dim_u),
        ))
        yref_e = copy.deepcopy(y_ref)
        for i in range(self.__num_window):
            self.__solver.set(i, "yref", yref)
        self.__solver.set(self.__num_window, "yref", yref_e)
        for i in range(self.__num_window):
            self.__solver.set(i, "x", x0)

    def solve(
        self,
        x_init: np.ndarray,
        y_tar: np.ndarray,
    ) -> Tuple[list, list]:
        # calculate target state
        y_ref_e = copy.deepcopy(y_tar)
        y_ref = np.concatenate((
            y_tar,
            np.zeros(2),
            np.zeros(self.__dim_u),
        ))

        # set the target state
        self.__solver.set(self.__num_window, "yref", y_ref_e)
        for i in range(self.__num_window):
            self.__solver.set(i, "yref", y_ref)

        ############################
        # solver
        ############################
        # set the initial state
        self.__solver.set(0, "lbx", x_init)
        self.__solver.set(0, "ubx", x_init)

        # solve and check the status of the solver
        status = self.__solver.solve()
        if status != 0:
            self.__solver.print_statistics()
            raise Exception(f"solver returned status {status}, exiting.")

        # get the control input and the state for the next step
        u_arr = [self.__solver.get(i, 'u') for i in range(self.__num_window)]
        x_arr = [self.__solver.get(i, 'x') for i in range(self.__num_window)]

        return u_arr, x_arr
