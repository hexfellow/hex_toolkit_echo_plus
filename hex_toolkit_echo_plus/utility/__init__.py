#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-07-06
################################################################

import os

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_interface import DataInterface as DataInterface
elif ROS_VERSION == '2':
    from .ros2_interface import DataInterface as DataInterface
else:
    raise ValueError("ROS_VERSION is not set")

from .obs_util import ObsUtil
from .ctrl_util import PdCtrl
from .ctrl_util import StanleyCtrl

__all__ = [
    "DataInterface",
    "ObsUtil",
    "PdCtrl",
    "StanleyCtrl",
]
