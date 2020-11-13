#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-

import sys
from rospkg import RosPack
sys.path.append(RosPack().get_path('canopen_communication') + "/modular/")
from modular_joint import Modular_joint
# import pdb

class T100(Modular_joint):
    """
    T100 control base on the canopen
    """

    def __init__(self, id, eds_file):
        self.__reduction_ratio = 480.0
        Modular_joint.__init__(self, id, eds_file, self.__reduction_ratio)
       