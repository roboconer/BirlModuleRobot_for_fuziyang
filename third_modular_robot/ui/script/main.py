#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: mian.py

@biref:　界面入口
"""

import sys
from PyQt5.QtWidgets import QApplication
from robot_choice_func import Robot_choice_func

#from get_fifo_relative_position_thread import Get_fifo_relative_position_thread, Send_fifo_feedback_translation_thread

#from modular_robot_control_func import Modular_robot_control_func

if __name__ == "__main__":

    app = QApplication(sys.argv)
    windows = Robot_choice_func() 
    windows.show()
    sys.exit(app.exec_())
    
    
    # while(1):
    #     all_position = __get_fifo_relative_position()
    #     tag = "1"
    #     __feedback_translation_tag(tag)
    
