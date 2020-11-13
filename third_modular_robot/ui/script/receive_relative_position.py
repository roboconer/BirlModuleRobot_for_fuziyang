#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: 
@file: receive_relative_position.py

@biref: 上位机抓夹机器人接收位置信息
"""

import rospy
import numpy as np
import math
from ui.msg import robot_feedback

from modular_robot_control import Ui_MainWindow_modular_robot
from path_point_recorder_func import Path_point_recorder_func
from gripper_control_func import Gripper_control_func
from zero_point_set_func import Zero_point_set_func
from robot_feedback_fun import Robot_feedback_fun
from receive_ros_command_func import Receive_ros_command_func
from robot_lowlevel_control_muti_thread import Robot_lowlevel_control_Muti_thread
from get_inverse_solution_thread import Get_inverse_solution_thread
from get_positive_solution_thread import Get_positive_solution_thread
from path_process import Path_process

from inverse_solution_client import Inverse_solution_client
from get_fifo_relative_position_thread import get_fifo_relative_position_thread
from get_fifo_relative_position_thread import relative_pose
from modular_robot_control_func import Modular_robot_control_func,  __robot_enabled_flag, __base_flag, __pos_joints

from PyQt5.QtWidgets import QMainWindow, QDesktopWidget, QMessageBox, QFileDialog, QWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QDoubleValidator,QIntValidator



class receive_relative_position(QMainWindow, Ui_MainWindow_modular_robot):

    
    # 关闭接受相对位姿
    close_receive_relative_joint_position_data  = pyqtSignal()

    # 转换相对位姿到base基座上
    translate_to_base_joint_position  = pyqtSignal(list)

    # 判断转换后是否正确
    determine_whether_translation_correct  = pyqtSignal()

    # 错误转换反馈
    wrong_translation_feedback = pyqtSignal()

    # 运行信号
    sin_run_operation   = pyqtSignal(bool)
    

    def __init__(self, which_robot):

        super(receive_relative_position, self).__init__()
        self.setupUi(self)
         
        # m 转为 mm
        self.__scale = 1000

        self._now_robot_pasitive_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._get_relative_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._goal_to_base_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_pisition_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.__center()  # 数据居中显示
        self.__lineEdit_default_Set()

        # 区别攀爬、爬壁
        self.__which_robot = which_robot

        # 判断机器人是否启动
        self.__robot_enabled_flag = False


    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())


    def __lineEdit_default_Set(self):

        # 接收笛卡尔位置显示
        self.lineEdit_38.setText(str(0))    # X mm
        self.lineEdit_39.setText(str(0))    # Y
        self.lineEdit_40.setText(str(0))    # Z
        self.lineEdit_41.setText(str(0))    # RX deg
        self.lineEdit_42.setText(str(0))    # RY
        self.lineEdit_43.setText(str(0))    # RZ

        # 到基座笛卡尔位置显示
        self.lineEdit_50.setText(str(0))    # X mm
        self.lineEdit_50.setText(str(0))    # X mm
        self.lineEdit_51.setText(str(0))    # Y
        self.lineEdit_52.setText(str(0))    # Z
        self.lineEdit_53.setText(str(0))    # RX deg
        self.lineEdit_54.setText(str(0))    # RY
        self.lineEdit_55.setText(str(0))    # RZ

        # 转换后是否正确
        self.lineEdit_72.setText(str(0))

    # 接收管道数据
    def receive_data(self):

        if self.__robot_enabled_flag and self.__which_robot == 0:
            self.__lowLevelControl = Robot_lowlevel_control_Muti_thread(self.__which_robot)
            
            self.relative_pose_process = Get_fifo_relative_position_thread(self)

            self.relative_pose_process.receive_relative__joint_position_data.connect(self._show_relative__joint_position_data)
            self.relative_pose_process.receive_relative__joint_position_data.connect(self._get_relative_position_data_)
            self.relative_pose_process.start()
            pass

    # 显示接收到的相对位姿数据
    def _show_relative__joint_position_data(self, data):

        self.lineEdit_38.setText(str(data[0]))
        self.lineEdit_39.setText(str(data[1]))
        self.lineEdit_40.setText(str(data[2]))
        self.lineEdit_41.setText(str(data[3]))
        self.lineEdit_42.setText(str(data[4]))
        self.lineEdit_43.setText(str(data[5]))
        pass
    
    # 存储接收到的相对位姿数据
    def _get_relative_position_data_(self, data):
        self._get_relative_position_data[0] = data[0]   # X mm
        self._get_relative_position_data[1] = data[1]   # Y mm
        self._get_relative_position_data[2] = data[2]   # Z mm
        self._get_relative_position_data[3] = data[3]   # RX degree
        self._get_relative_position_data[4] = data[4]   # RY degree
        self._get_relative_position_data[5] = data[5]   # RZ degree
        pass
    
    # 转换 接收到的位姿到基座标的表示
    def translate_relative_position_to_base(self, data):
        
        if self.__robot_enabled_flag and self.__which_robot == 0:     
            
            if len(get_fifo_relative_position_thread.relative_pose) < 0:
                print "the fifo read end or error."
            else
                # 获得机器人当前末端的位姿
                self.__get_positive_solution = Get_positive_solution_thread()
                self.__get_positive_solution.sin_positive_solution.connect(self.get_now_positive_data)

                # 目标点到基座标的位姿表示
                self._goal_to_base_position_data[0] = self._get_relative_position_data[0] + self.get_now_positive_data[0]* self.__scale # goal to base: X
                self._goal_to_base_position_data[1] = self._get_relative_position_data[1] + self.get_now_positive_data[1]* self.__scale # goal to base: Y
                self._goal_to_base_position_data[2] = self._get_relative_position_data[2] + self.get_now_positive_data[2]* self.__scale # goal to base: Z
                self._goal_to_base_position_data[3] = self._get_relative_position_data[3] + self.get_now_positive_data[3] # goal to base: RX
                self._goal_to_base_position_data[4] = self._get_relative_position_data[4] + self.get_now_positive_data[4] # goal to base: RY
                self._goal_to_base_position_data[5] = self._get_relative_position_data[5] + self.get_now_positive_data[5] # goal to base: RZ
                
                # 将目标点到基座标的位姿显示在ui上
                self.translate_to_base_joint_position.connect(self.show_goal_to_base_position_data)

                self.__data_modular_control_ = Modular_robot_control_func(QMainWindow,Ui_MainWindow_modular_robot)
                self._current_joint_position = self.__data_modular_control_.__pos_joint  # position I1, T2, T3, T4, I5
                
                # 调用机器人逆解函数，获得关节运动情况(含逆解正确与否的标志位)
                [_joint_position_command, _joint_velocity_command, inverse_solution_tag] = Inverse_solution_client(1,  __base_flag, _goal_to_base_position_data, [0.004, 0.002, 0.002, 0.002, 0.004], _current_joint_position)
                
                # 在ui界面显示逆解转换成功与否
                self.determine_whether_translation_correct.emit(inverse_solution_tag)
                self.determine_whether_translation_correct.connect(self.show_determine_whether_translation_correct_flag)
                # or: self.lineEdit_72.setText(inverse_solution_tag)
            
                # 向另一个管道发送逆解转换成功与否
                self.relative_pose_process = get_fifo_relative_position_thread(self)
                self.relative_pose_process.__feedback_translation_tag(self, inverse_solution_tag)
        pass

    def run_translation_to_base(self, _joint_position_command, _joint_velocity_command):

        for i in range(len(_joint_position_command)):
            temp_pos_command[i] = Modular_robot_control_func.__user_data_to_motor(_joint_position_command[i], self.__zero_pos_joints[i], self.__direction_joints[i])
        pass
    
    # 获得机器人当前末端的位姿
    def get_now_positive_data(self, data):

        self._now_robot_pasitive_position[0] = data[0]
        self._now_robot_pasitive_position[1] = data[1]
        self._now_robot_pasitive_position[2] = data[2]
        self._now_robot_pasitive_position[3] = data[3]
        self._now_robot_pasitive_position[4] = data[4]
        self._now_robot_pasitive_position[5] = data[5]
        pass

    # 将目标点到基座标的位姿显示在ui上
    def show_goal_to_base_position_data(self, data):
        self.lineEdit_50.setText(str(data[0]))
        self.lineEdit_51.setText(str(data[1]))
        self.lineEdit_52.setText(str(data[2]))
        self.lineEdit_53.setText(str(data[3]))
        self.lineEdit_54.setText(str(data[4]))
        self.lineEdit_55.setText(str(data[5]))
        self.translate_to_base_joint_position.emit(_goal_to_base_position_data)
        pass

    # 在ui界面显示逆解转换成功与否   
    def show_determine_whether_translation_correct_flag(self, data):
        self.lineEdit_72.setText(data)
        pass

    
    # ##
    # class _robot_matrix:
        
    #     PI_RAD = 0.0174532925199  # 角度转换为弧度参数 (3.14/180)
    #     R11 = 0.0    
    #     R12 = 0.0
    #     R13 = 0.0
    #     R21 = 0.0
    #     R22 = 0.0
    #     R23 = 0.0
    #     R31 = 0.0
    #     R32 = 0.0
    #     R33 = 0.0
    #     Px  = 0.0
    #     Py  = 0.0
    #     Pz  = 0.0
    #     Ps  = 1.0

    # # 7个值： 0 sin(x) cos(x) sin(y) cos(y) sin(z) cos(z)
    # triangle_value_of_angle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    

    # def __init__(self, name, data)
    #     self.name = name
    #     self.data = data

    #     triangle_value_of_angle[1] = sin(data[3]*PI_RAD)  # sin(Rx)
    #     triangle_value_of_angle[2] = cos(data[3]*PI_RAD)  # cos(Ry)
    #     triangle_value_of_angle[3] = sin(data[4]*PI_RAD)  # sin(Ry)
    #     triangle_value_of_angle[4] = cos(data[4]*PI_RAD)  # cos(Ry)
    #     triangle_value_of_angle[5] = sin(data[5]*PI_RAD)  # sin(Rz)
    #     triangle_value_of_angle[6] = cos(data[5]*PI_RAD)  # cos(Rz)
    
    # def rotation_matrix(self):
    #     R11 = triangle_value_of_angle[2] * triangle_value_of_angle[4]
    #     R12 = triangle_value_of_angle[2] * triangle_value_of_angle[3] * triangle_value_of_angle[5] - triangle_value_of_angle[1] * triangle_value_of_angle[6]
    #     R13 = triangle_value_of_angle[2] * triangle_value_of_angle[3] * triangle_value_of_angle[6] + triangle_value_of_angle[1] * triangle_value_of_angle[5]
    #     R21 = triangle_value_of_angle[1] * triangle_value_of_angle[4] 
    #     R22 = triangle_value_of_angle[2] * triangle_value_of_angle[6] + triangle_value_of_angle[1] * triangle_value_of_angle[3] * triangle_value_of_angle[5]
    #     R23 = triangle_value_of_angle[1] * triangle_value_of_angle[3] * triangle_value_of_angle[6] - triangle_value_of_angle[5] * triangle_value_of_angle[2]
    #     R31 = -triangle_value_of_angle[3]
    #     R32 = triangle_value_of_angle[4] * triangle_value_of_angle[5]
    #     R33 = triangle_value_of_angle[4] * triangle_value_of_angle[6]