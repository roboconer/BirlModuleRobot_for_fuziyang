#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: modular_robot_control_func.py

@biref:上位机主窗口，包含关节控制，路径离线控制
"""
import rospy
import sys
from rospkg import RosPack
from ui.msg import robot_feedback

from modular_robot_control import Ui_MainWindow_modular_robot
from path_point_recorder_func import Path_point_recorder_func
from gripper_control_func import Gripper_control_func
from zero_point_set_func import Zero_point_set_func
from robot_feedback_fun import Robot_feedback_fun
from receive_ros_command_func import Receive_ros_command_func
from robot_lowlevel_control_muti_thread import Robot_lowlevel_control_Muti_thread
from get_inverse_solution_thread import Get_inverse_solution_thread
from path_process import Path_process

from PyQt5.QtWidgets import QMainWindow, QDesktopWidget, QMessageBox, QFileDialog, QWidget
from PyQt5.QtCore import pyqtSignal, QMutex
from PyQt5.QtGui import QDoubleValidator,QIntValidator

# 新增的引用文件 RosPack().get_path('third_modular_robot') + "/birl_modular_robot/script/"
sys.path.append('~/ros/BirlModuleRobot/src/third_modular_robot/birl_modular_robot/script/') 
# export PYTHONPATH = ~/ros/BirlModuleRobot/src/third_modular_robot/birl_modular_robot/script/
from inverse_solution_client import Inverse_solution_client

# from get_fifo_relative_position_thread import __get_fifo_relative_position, __feedback_translation_tag
from get_fifo_relative_position_thread import Get_fifo_relative_position_thread, Translate_goal_to_base_thread
from get_positive_solution_thread import Get_positive_solution_thread


# from string import atof

class Modular_robot_control_func(QMainWindow,Ui_MainWindow_modular_robot):


    # 窗口传递信号
    sin_sent_data_to_WindowsPathRecoder = pyqtSignal(list)
    sin_sent_data_to_windowsSetZero     = pyqtSignal(list)

    # 关闭窗口信号
    sin_close_windowsPathRecoder    = pyqtSignal()
    sin_close_windowsSetZero        = pyqtSignal()
    sin_close_windowsGripper        = pyqtSignal()
    sin_close_windowsFeedback       = pyqtSignal()
    
    # 停止、急停、运行信号
    sin_stop_robot_operation            = pyqtSignal()
    sin_quickStop_robot_operation       = pyqtSignal()
    sin_halt_robot_operation            = pyqtSignal(bool)

    # 关节位置命令信号
    sin_joint_position = pyqtSignal(list)
    # 关节速度命令信号
    sin__joint_velocity = pyqtSignal(list)

    # 夹持器信号
    sin_G0_command = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    # 路径信号
    sin_path_command = pyqtSignal(list)

    # 反馈数据显示信号
    sin_display_feedback_data = pyqtSignal(list)
    # 更新爪子基座
    sin_update_gripper_base = pyqtSignal(bool)

    # 机器人各关节状态
    # position I1, T2, T3, T4, I5
    __pos_joints = [0, 0, 0, 0, 0]

    # 将相对位姿转为base坐标下的位姿信号
    sin_display_goal_to_base_data = pyqtSignal(list)

    # 运行相对位姿到base坐标下的关节指令
    run_goal_to_base_motor_command = pyqtSignal(list)



    def  __init__(self,which_robot):
        super(Modular_robot_control_func,self).__init__()
        self.setupUi(self)

        self.__center()
        self.__input_range_limit()
        self.__lineEdit_default_Set()

        self.__string_robot_Enabled = "机器人\n使能成功"
        self.__string_robot_NotEnabled = "机器人\n未使能"
        self.__string_robot_Enable_ing = "机器人\n使能中"

        # 窗口
        self.__window_gripper_control = QWidget()
        self.__window_path_point_record = QWidget()
        self.__window_zero_point_set = QWidget()
        self.__window_robot_state_feedback = QWidget()
        self.__window_receive_ros_robot_command = QWidget()
        # 判断窗口是否打开
        self.__window_gripper_control_flag = False
        self.__window_path_point_record_flag = False
        self.__window_zero_point_set_flag = False
        self.__window_robot_state_feedback_flag = False
        self.__window_receive_ros_robot_command_flag = False

        # 区别攀爬、爬壁
        self.__which_robot = which_robot

        # 判断机器人是否启动
        self.__robot_enabled_flag = False
        # 判断是否为机器人启动中，防止启动过程重复启动
        self.__robot_starting = False
        self.label.setText(self.__string_robot_NotEnabled)

        # 离线路径点集合
        self.__pos_joints_path_array = []
        self.__old_path_array = []

        # 离线路径速度
        self.__max_path_velocity = 5
        self.lineEdit.setText(str(self.__max_path_velocity))

        # 关节速度命令
        self.__joint_velocity = 3
        self.lineEdit_12.setText(str(self.__joint_velocity))
        self.lineEdit_23.setText(str(self.__joint_velocity))

        # 位置模式界面
        self.___joint_velocity_command_posMode = [0,0,0,0,0]
        # 速度模式界面
        self.___joint_velocity_command_VelMode = [0,0,0,0,0]

        # 关节位置命令
        self.__joint_position_command = [0,0,0,0,0]

        # 关节零点 I1, T2, T3, T4, I5
        self.__zero_pos_joints = [0,0,0,0,0]

        # 关节是否正方向(1:正;-1:反)I1, T2, T3, T4, I5
        self.__direction_joints = [1,1,1,1,1]

        # 默认机器人正逆运动学基座为G0
        self.__base_flag = True

        # 机器人反馈数据显示
        self.__robot_state_display_flag = False

        # 判断机器人是否通过ros反馈状态消息  
        self.__ros_feedback_flag = False

        # 菜单栏信号槽函数链接
        self.action.triggered.connect(self.__open_gripper_control)
        self.action_2.triggered.connect(self.__open_path_point_record)
        self.action_3.triggered.connect(self.__open_set_zero_point)
        self.action_11.triggered.connect(self.__open_robot_state_feedback)
        self.action_4.triggered.connect(self.__about)
        self.action_5.triggered.connect(self.__quit)
        self.action_8.triggered.connect(self.__auto_gripper_pole)
        self.action_6.triggered.connect(self.__ros_state_feedback_Set)
        self.action_10.triggered.connect(self.__about)
        self.action_12.triggered.connect(self.__open_ros_command)
        # end

        # ros 反馈初始化
        rospy.init_node("modular_robot_lowlevel_control", anonymous=True)
        self.__publisher = rospy.Publisher("/low_level/joints_point_feedback",robot_feedback,queue_size = 10)
        self.__ros_feedback_msg = robot_feedback()
        # 关节位置
        self.__ros_feedback_msg.feedbackPosData = [0 ,0 ,0 ,0 ,0]
        # 关节速度
        self.__ros_feedback_msg.feedbackVelData = [0 ,0 ,0 ,0 ,0]
        # 关节电流
        self.__ros_feedback_msg.feedbackCurrData = [0 ,0 ,0 ,0 ,0]
        # end

# 新增变量
        # m 转为 mm
        self.__scale = 1000

        self._now_robot_pasitive_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 级
        self._get_relative_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._goal_to_base_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_pisition_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._joint_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_to_base_motor_temp_position_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_to_base_motor_temp_velocity_command = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_to_base_tag_ = " "
         # 线程同步锁
        self.__mutex = QMutex()

    # 窗口位于屏幕中心
    def __center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    # 启动机器人
    def start_robot(self):

        if (not self.__robot_enabled_flag) and (not self.__robot_starting):
            self.__robot_starting = True
            # 底层控制
            self.__lowLevelControl = Robot_lowlevel_control_Muti_thread(self.__which_robot)                
            self.__lowLevelControl.sin_robot_start_compelete.connect(self.__robot_start_compelete)
            self.__lowLevelControl.sin_robot_start_error.connect(self.start_robot_error)
            self.__lowLevelControl.sin_robot_stop_compelete.connect(self.__robot_stop_compelete)
            self.__lowLevelControl.sin_feedback.connect(self.__update_feeback_joint_position)

            self.sin_quickStop_robot_operation.connect(self.__lowLevelControl.robot_quick_stop)
            self.sin_stop_robot_operation.connect(self.__lowLevelControl.robot_stop)
            self.sin_halt_robot_operation.connect(self.__lowLevelControl.robot_halt)
            self.sin_joint_position.connect(self.__lowLevelControl.set_jointPosition)
            self.sin__joint_velocity.connect(self.__lowLevelControl.set_jointVelocity)
            self.sin_path_command.connect(self.__lowLevelControl.path_command)

            self.label.setText(self.__string_robot_Enable_ing)
            if not self.__lowLevelControl.isRunning():
                self.__lowLevelControl.start()

    # 机器人启动错误信号槽函数
    def start_robot_error(self):
        self.__robot_enabled_flag = False
        self.__robot_starting = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)
        QMessageBox.about(self,'错误',"\n\n机器人启动失败,请检查线路连接是否正确.\n\n"   )

    # 机器人停止槽函数
    def stop_robot(self):
        if self.__robot_enabled_flag:
            self.sin_stop_robot_operation.emit()
        pass

    # 机器人急停槽函数
    def quick_stop(self):
        if self.__robot_enabled_flag:
            self.sin_quickStop_robot_operation.emit()
            pass
        pass

    # 机器人暂停槽函数
    def halt(self,data):
        if self.__robot_enabled_flag:
            self.sin_halt_robot_operation.emit(data)
        pass
    
    # 机器人启动成功反馈槽函数
    def __robot_start_compelete(self):
        # print "complete"
        self.__robot_enabled_flag = True
        self.label.setText(self.__string_robot_Enabled)

    # 机器人关闭成功反馈槽函数
    def __robot_stop_compelete(self):
        self.__robot_enabled_flag = False
        self.__robot_starting = False
        del self.__lowLevelControl
        self.label.setText(self.__string_robot_NotEnabled)

    #################### 关节空间位置控制界面 关节控制槽函数 ###################################################
    def joint_i1_command(self):
        if self.__robot_enabled_flag:  
            # print "i1_command"
            self.__joint_position_command[0] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_7.text())), \
                                                self.__zero_pos_joints[0],  \
                                                self.__direction_joints[0]  )
            self.___joint_velocity_command_posMode[0] = self.__joint_velocity * self.__direction_joints[0]
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t2_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[1] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_8.text())), \
                                                self.__zero_pos_joints[1],  \
                                                self.__direction_joints[1]  )
            self.___joint_velocity_command_posMode[1] = self.__joint_velocity  * self.__direction_joints[1]
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t3_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[2] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_9.text())), \
                                                self.__zero_pos_joints[2],  \
                                                self.__direction_joints[2]  )
            self.___joint_velocity_command_posMode[2] = self.__joint_velocity * self.__direction_joints[2]
            self.__joint_pos_command(self.__joint_position_command)

    def joint_t4_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[3] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_10.text())), \
                                                self.__zero_pos_joints[3],  \
                                                self.__direction_joints[3]  )
            self.___joint_velocity_command_posMode[3] = self.__joint_velocity * self.__direction_joints[3]
            self.__joint_pos_command(self.__joint_position_command)

    def joint_i5_command(self):
        if self.__robot_enabled_flag:
            self.__joint_position_command[4] =  Modular_robot_control_func.__user_data_to_motor(   \
                                                float(str(self.lineEdit_11.text())), \
                                                self.__zero_pos_joints[4],  \
                                                self.__direction_joints[4]  )
            self.___joint_velocity_command_posMode[4] = self.__joint_velocity * self.__direction_joints[4]
            self.__joint_pos_command(self.__joint_position_command)

    def __joint_pos_command(self,data):
        self.sin_joint_position.emit([data, self.___joint_velocity_command_posMode])

    #################### 关节空间位置控制界面 关节控制槽函数 end ###################################################

    #################### 关节空间速度控制界面 关节控制槽函数 #######################################################
    def joint_i1_vel_command(self,data):
        if not data:
            self.___joint_velocity_command_VelMode[0] = 0
        else:
            self.___joint_velocity_command_VelMode[0] = self.__joint_velocity * self.__direction_joints[0]
        self.sin__joint_velocity.emit(self.___joint_velocity_command_VelMode)

    def joint_t2_vel_command(self,data):
        if not data:
            self.___joint_velocity_command_VelMode[1] = 0
        else:
            self.___joint_velocity_command_VelMode[1] = self.__joint_velocity * self.__direction_joints[1]
        self.sin__joint_velocity.emit(self.___joint_velocity_command_VelMode)
        # print self.___joint_velocity_command_VelMode

    def joint_t3_vel_command(self,data):
        if not data:
            self.___joint_velocity_command_VelMode[2] = 0
        else:
            self.___joint_velocity_command_VelMode[2] = self.__joint_velocity * self.__direction_joints[2]
        self.sin__joint_velocity.emit(self.___joint_velocity_command_VelMode)

    def joint_t4_vel_command(self,data):
        if not data:
            self.___joint_velocity_command_VelMode[3] = 0
        else:
            self.___joint_velocity_command_VelMode[3] = self.__joint_velocity * self.__direction_joints[3]
        self.sin__joint_velocity.emit(self.___joint_velocity_command_VelMode)

    def joint_i5_vel_command(self,data):
        if not data:
            self.___joint_velocity_command_VelMode[4] = 0
        else:
            self.___joint_velocity_command_VelMode[4] = self.__joint_velocity * self.__direction_joints[4]
        self.sin__joint_velocity.emit(self.___joint_velocity_command_VelMode)  
    #################### 关节空间速度控制界面 关节控制槽函数  end ###################################################

    # 获取逆解槽函数
    def get_inverse_solution(self):
        
        # mm单位 转换到 m单位
        scale = 0.001

        temp_descartes_postion_command = []
        temp_descartes_postion_command.append(float(str(self.lineEdit_26.text())) * scale)  # X
        temp_descartes_postion_command.append(float(str(self.lineEdit_27.text())) * scale)  # Y
        temp_descartes_postion_command.append(float(str(self.lineEdit_28.text())) * scale)  # Z
        temp_descartes_postion_command.append(float(str(self.lineEdit_29.text())))  # RX
        temp_descartes_postion_command.append(float(str(self.lineEdit_30.text())))  # RY
        temp_descartes_postion_command.append(float(str(self.lineEdit_31.text())))  # RZ

        temp_descartes_velocity_command = []
        temp_descartes_velocity_command.append(float(str(self.lineEdit_34.text())) * scale)  # X
        temp_descartes_velocity_command.append(float(str(self.lineEdit_37.text())) * scale)  # Y
        temp_descartes_velocity_command.append(float(str(self.lineEdit_32.text())) * scale)  # Z
        temp_descartes_velocity_command.append(float(str(self.lineEdit_33.text())))  # RX
        temp_descartes_velocity_command.append(float(str(self.lineEdit_36.text())))  # RY
        temp_descartes_velocity_command.append(float(str(self.lineEdit_35.text())))  # RZ

        # print temp_descartes_postion_command
        # print temp_descartes_velocity_command
        # print self.__pos_joints
        # print self.__which_robot
        # print self.__base_flag

        self.__getInverseSolu = Get_inverse_solution_thread(self.__which_robot, self.__base_flag, temp_descartes_postion_command, temp_descartes_velocity_command, self.__pos_joints)
        self.__getInverseSolu.sin_inverse_solution.connect(self.__show_the_inverse_solution)
        self.__getInverseSolu.start()

    # 显示运动学逆解结果
    def __show_the_inverse_solution(self,data):
        # joint position command
        self.lineEdit_58.setText(str(data[0][0]))       # I1 deg
        self.lineEdit_61.setText(str(data[0][1]))       # T2
        self.lineEdit_56.setText(str(data[0][2]))       # T3
        self.lineEdit_57.setText(str(data[0][3]))       # T4
        self.lineEdit_60.setText(str(data[0][4]))       # I5
        # joint velocity command 
        self.lineEdit_64.setText(str(data[1][0]))        # I1 deg/s
        self.lineEdit_67.setText(str(data[1][1]))        # T2
        self.lineEdit_62.setText(str(data[1][2]))        # T3
        self.lineEdit_63.setText(str(data[1][3]))        # T4
        self.lineEdit_66.setText(str(data[1][4]))        # I5
        pass

    # 运行运动学逆解结果
    def descartes_command(self):
        # print "descartes_command"
        temp_velocity_command = [   float(str(self.lineEdit_64.text())) * self.__direction_joints[0],   \
                                    float(str(self.lineEdit_67.text())) * self.__direction_joints[1],   \
                                    float(str(self.lineEdit_62.text())) * self.__direction_joints[2],   \
                                    float(str(self.lineEdit_63.text())) * self.__direction_joints[3],   \
                                    float(str(self.lineEdit_66.text())) * self.__direction_joints[4]    ]

        temp_pos_command = [    float(str(self.lineEdit_58.text())),    \
                                float(str(self.lineEdit_61 .text())),   \
                                float(str(self.lineEdit_56.text())),    \
                                float(str(self.lineEdit_57.text())),    \
                                float(str(self.lineEdit_60.text()))     ]
        
        for i in range(len(temp_pos_command)):
            temp_pos_command[i] = Modular_robot_control_func.__user_data_to_motor(temp_pos_command[i],self.__zero_pos_joints[i],self.__direction_joints[i])

        # print temp_pos_command
        # print temp_velocity_command
        self.sin_joint_position.emit([temp_pos_command, temp_velocity_command])

    # 关节空间位置控制界面、速度控制界面速度设置
    def joint_velocity_set_posMode(self):
        self.__joint_velocity = abs(float(str(self.lineEdit_12.text())))

    def joint_velocity_set_velMode(self):
        self.__joint_velocity = float(str(self.lineEdit_23.text()))

    # 离线轨迹载入文件槽函数
    def load_point_file(self):

        # file_path = QFileDialog.getOpenFileName(self, "载入文件", "~/")
        # if file_path[0]:
        #     f = open(file_path[0], "r")
        #     data = f.read()
        #     f.close()
        #     self.textEdit.setText(data)

        self.__pos_joints_path_array = []
        temp_data = self.textEdit.toPlainText()
        temp_path_process = Path_process(self.__zero_pos_joints, self.__direction_joints, self.__max_path_velocity)

        try:
            self.__pos_joints_path_array = temp_path_process.get_trajectory(temp_data) 
            QMessageBox.about(self,'通知','\n       路径点载入成功!!        \n')
        except:
            QMessageBox.about(self,'错误','\n       路径点格式错误!!        \n')
            return 
        # print self.__pos_joints_path_array

    # 设置离线轨迹最大速度
    def set_path_max_velovity(self):
        self.__max_path_velocity = int(str(self.lineEdit.text()))
        # print self.__max_path_velocity

    # 运行离线轨迹槽函数
    def operate_point_data(self):

        if self.__robot_enabled_flag:
            if self.__pos_joints_path_array != self.__old_path_array:
                self.__old_path_array = self.__pos_joints_path_array
                self.sin_path_command.emit(self.__pos_joints_path_array)

    # 打开夹持器控制界面
    def __open_gripper_control(self):

        if self.__which_robot == 0:
            if not self.__window_gripper_control_flag:
                self.__window_gripper_control_flag = True
                self.__window_gripper_control = Gripper_control_func()
                temp = self.frameGeometry()
                self.__window_gripper_control.move(temp.right(),temp.top())
                
                self.__window_gripper_control.sin_open_or_close_gripper0.connect(self.__get_G0_torque_from_windowsGripperControl)
                self.__window_gripper_control.sin_open_or_close_gripper6.connect(self.__get_G6_torque_from_windowsGripperControl)
                self.__window_gripper_control.sin_close.connect(self.__open_gripper_control_close_flag)

                self.sin_close_windowsGripper.connect(self.__window_gripper_control.close_windows)
                
                if self.__robot_enabled_flag:
                    self.sin_G0_command.connect(self.__lowLevelControl.G0_command)
                    self.sin_G6_command.connect(self.__lowLevelControl.G6_command)
                    self.__window_gripper_control.sin_open_gripper_feedback.connect(self.__lowLevelControl.open_gripper_feedback)
                    
                    self.__lowLevelControl.sin_gripper_feedback.connect(self.__window_gripper_control.display)
                self.__window_gripper_control.show()
                self.__window_gripper_control.open_gripper_current_feedback()

    def __open_gripper_control_close_flag(self):
        self.__window_gripper_control_flag = False

    # 打开示教点记录界面
    def __open_path_point_record(self):
        if not self.__window_path_point_record_flag:
            self.__window_path_point_record_flag = True
            self.__window_path_point_record = Path_point_recorder_func()
            temp = self.frameGeometry()
            temp_height = self.__window_path_point_record.height()
            self.__window_path_point_record.move(temp.right(), temp.top() + temp_height)
            
            self.__window_path_point_record.sin_request_pos_joints.connect(self.__sent_data_to_windowsPathRecorder)
            self.__window_path_point_record.sin_close.connect(self.__open_path_point_record_close_flag)
            self.sin_sent_data_to_WindowsPathRecoder.connect(self.__window_path_point_record.receive_pos_joints)
            self.sin_close_windowsPathRecoder.connect(self.__window_path_point_record.close_windows)
            self.__window_path_point_record.show()

    def __open_path_point_record_close_flag(self):
        self.__window_path_point_record_flag = False

    # 打开零点设置界面
    def __open_set_zero_point(self):
        if not self.__window_zero_point_set_flag:
            self.__window_zero_point_set_flag = True
            self.__window_zero_point_set = Zero_point_set_func(self.__zero_pos_joints, self.__direction_joints)
            temp = self.frameGeometry()
            temp_width = self.__window_zero_point_set.width()
            self.__window_zero_point_set.move(temp.left() - temp_width, temp.top())
            self.__window_zero_point_set.sin_close.connect(self.__open_set_zero_point_close_flag)
            self.__window_zero_point_set.sin_set_zero_point.connect(self.__get_data_from_windowsSetZeroPoint)
            self.__window_zero_point_set.sin_get_actual_joints_pos.connect(self.__sent_data_to_windowsSetZero)
            self.sin_sent_data_to_windowsSetZero.connect(self.__window_zero_point_set.receive_actual_joint_point)
            self.sin_close_windowsSetZero.connect(self.__window_zero_point_set.close_windows)
            self.__window_zero_point_set.show()
            pass

    def __open_set_zero_point_close_flag(self):
        self.__window_zero_point_set_flag = False
    
    # 打开机器人状态反馈界面
    def __open_robot_state_feedback(self):

        if not self.__window_robot_state_feedback_flag:
            self.__window_robot_state_feedback_flag = True
            self.__window_robot_state_feedback = Robot_feedback_fun(self.__which_robot)
            if self.__robot_enabled_flag:
                self.sin_display_feedback_data.connect(self.__window_robot_state_feedback.display)
                self.sin_update_gripper_base.connect(self.__window_robot_state_feedback.update_gripper)
                self.__window_robot_state_feedback.sin_open_robot_state_feedback.connect(self.__open_robot_state_feedback_signal)
            self.__window_robot_state_feedback.sin_close.connect(self.__open_robot_state_feedback_close_flag)
            self.sin_close_windowsFeedback.connect(self.__window_robot_state_feedback.close_windows)
            temp = self.frameGeometry()
            temp_width = self.__window_robot_state_feedback.width()
            temp_height = self.__window_robot_state_feedback.height()
            self.__window_robot_state_feedback.move(temp.left() - temp_width, temp.top() + temp_height)
            self.__window_robot_state_feedback.show()
            self.__window_robot_state_feedback.open_robot_state_feedback()
    
    def __open_robot_state_feedback_close_flag(self):
        self.__window_robot_state_feedback_flag = False

    # 打开 接收ros command 界面
    def __open_ros_command(self):

        if not self.__window_receive_ros_robot_command_flag:
            self.__window_receive_ros_robot_command_flag = True
            self.__window_receive_ros_robot_command = Receive_ros_command_func()
            self.__window_receive_ros_robot_command.sin_sent_ros_command.connect(self.__get_ros_command_data)
            self.__window_receive_ros_robot_command.sin_close.connect(self.__open_ros_command_close_flag)

            temp = self.frameGeometry()
            self.__window_receive_ros_robot_command.move(temp.right(),temp.top())
            self.__window_receive_ros_robot_command.show()

    def __open_ros_command_close_flag(self):
        self.__window_receive_ros_robot_command_flag = False

    # 获取ros command， 发送至底层
    def __get_ros_command_data(self, data):
        self.__pos_joints_path_array = data
        if self.__old_path_array != self.__pos_joints_path_array:
            self.__old_path_array = self.__pos_joints_path_array
            self.sin_path_command.emit(self.__pos_joints_path_array)
        pass

    # 打开机器人状态反馈界面，开始显示反馈数据
    def __open_robot_state_feedback_signal(self, data):
        self.__robot_state_display_flag = data

    # 自主抓夹
    def __auto_gripper_pole(self):       
        pass
    
    ####################### 获取夹持器界面控制数据，并发送至底层 ###########################
    def __get_G0_torque_from_windowsGripperControl(self,data):
        if self.__robot_enabled_flag:
            self.sin_G0_command.emit(data)

    def __get_G6_torque_from_windowsGripperControl(self,data):
        if self.__robot_enabled_flag:
            self.sin_G6_command.emit(data)
    ####################### 获取夹持器界面控制数据，并发送至底层 end ########################

    # 界面退出
    def __quit(self):

        self.sin_close_windowsGripper.emit()
        self.sin_close_windowsPathRecoder.emit()
        self.sin_close_windowsSetZero.emit()
        self.sin_close_windowsFeedback.emit()

        self.close()
        pass
    
    def __about(self):
        QMessageBox.about(self,'关于',"模块化机器人上位机 V1.0.    \
                                     \n\nAuthor:           \
                                      \n\nMail:")
    
    # 更新关节数据 至示教记录界面
    def __sent_data_to_windowsPathRecorder(self):
        self.sin_sent_data_to_WindowsPathRecoder.emit(self.__pos_joints)

    # 更新关节数据 至零点设置界面
    def __sent_data_to_windowsSetZero(self):
        self.sin_sent_data_to_windowsSetZero.emit(self.__pos_joints)
    
    # 获取零点，更新零点
    def __get_data_from_windowsSetZeroPoint(self,data):
        for i in range(len(data[0])):
            self.__direction_joints[i] = data[1][i]
            if self.__direction_joints[i] == 1 or self.__zero_pos_joints[i] == 0:
                self.__zero_pos_joints[i] += data[0][i]
            else:
                self.__zero_pos_joints[i] -= data[0][i]

    # 根据零点、关节方向， 计算关节命令
    @staticmethod
    def __user_data_to_motor(command_pos, zero_pos, direction):
        """
        brief:  关节方向 * 用户值 + 零点    
        """
        return round((direction * command_pos + zero_pos) , 3)

    # 更新机器人关节状态
    def __update_feeback_joint_position(self,data):

        for i in range(len(self.__pos_joints)):
            self.__pos_joints[i] = self.__direction_joints[i] * ( data[0][i] - self.__zero_pos_joints[i] )

        if self.__robot_state_display_flag:
            # 关节位置、速度、电流
            self.sin_display_feedback_data.emit([self.__pos_joints, data[1], data[2]])
        
        if self.__ros_feedback_flag:
            self.__ros_feedback_msg.feedbackPosData = self.__pos_joints
            self.__ros_feedback_msg.feedbackVelData = data[1]
            self.__ros_feedback_msg.feedbackCurrData = data[2]
            self.__ros_feedback_msg.timeHeader.stamp = rospy.Time.now()
            self.__publisher.publish(self.__ros_feedback_msg)

    # 输入框　输入限制
    def __input_range_limit(self):

        # 离线轨迹最大速度限制
        pIntValidator = QIntValidator(self)
        pIntValidator.setRange(1,30)
        self.lineEdit.setValidator(pIntValidator)

        pDoubleValidator_I = QDoubleValidator(self)
        pDoubleValidator_I.setRange(-360, 360)
        pDoubleValidator_I.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_I.setDecimals(2)
        # I1,I5 位置范围
        self.lineEdit_7.setValidator(pDoubleValidator_I)
        self.lineEdit_11.setValidator(pDoubleValidator_I)

        pDoubleValidator_T = QDoubleValidator(self)
        pDoubleValidator_T.setRange(-120, 120)
        pDoubleValidator_T.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_T.setDecimals(2)
        # T2 T3 T4 位置范围 
        self.lineEdit_8.setValidator(pDoubleValidator_T)
        self.lineEdit_9.setValidator(pDoubleValidator_T)
        self.lineEdit_10.setValidator(pDoubleValidator_T)

        pDoubleValidator_V = QDoubleValidator(self)
        pDoubleValidator_V.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_V.setDecimals(2)
        # 关节速度范围
        self.lineEdit_12.setValidator(pDoubleValidator_V)
        pDoubleValidator_V.setRange(-30, 30)
        self.lineEdit_23.setValidator(pDoubleValidator_V)

        # 笛卡尔关节控制输入限制
        pDoubleValidator_descartes_pos_XYZ = QDoubleValidator(self)
        pDoubleValidator_descartes_pos_XYZ.setRange(-1000,1000)
        pDoubleValidator_descartes_pos_XYZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_pos_XYZ.setDecimals(2)
        self.lineEdit_26.setValidator(pDoubleValidator_descartes_pos_XYZ)
        self.lineEdit_27.setValidator(pDoubleValidator_descartes_pos_XYZ)
        self.lineEdit_28.setValidator(pDoubleValidator_descartes_pos_XYZ)

        pDoubleValidator_descartes_pos_RXRYRZ = QDoubleValidator(self)
        pDoubleValidator_descartes_pos_RXRYRZ.setRange(-180,180)
        pDoubleValidator_descartes_pos_RXRYRZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_pos_RXRYRZ.setDecimals(2)
        self.lineEdit_29.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)
        self.lineEdit_30.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)
        self.lineEdit_31.setValidator(pDoubleValidator_descartes_pos_RXRYRZ)

        pDoubleValidator_descartes_vel_XYZ = QDoubleValidator(self)
        pDoubleValidator_descartes_vel_XYZ.setRange(-100,100) 
        pDoubleValidator_descartes_vel_XYZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_vel_XYZ.setDecimals(2)
        self.lineEdit_34.setValidator(pDoubleValidator_descartes_vel_XYZ)
        self.lineEdit_37.setValidator(pDoubleValidator_descartes_vel_XYZ)
        self.lineEdit_32.setValidator(pDoubleValidator_descartes_vel_XYZ)

        pDoubleValidator_descartes_vel_RXRYRZ = QDoubleValidator(self)
        pDoubleValidator_descartes_vel_RXRYRZ.setRange(-30,30)
        pDoubleValidator_descartes_vel_RXRYRZ.setNotation(QDoubleValidator.StandardNotation)
        pDoubleValidator_descartes_vel_RXRYRZ.setDecimals(2)
        self.lineEdit_33.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)
        self.lineEdit_36.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)
        self.lineEdit_35.setValidator(pDoubleValidator_descartes_vel_RXRYRZ)

    # 界面突出处理信号
    def closeEvent(self, event):
        result = QMessageBox.question(self, "模块化机器人上位机", '\n       退出?        \n', QMessageBox.Yes | QMessageBox.No)
        if(result == QMessageBox.Yes):
            self.sin_close_windowsGripper.emit()
            self.sin_close_windowsPathRecoder.emit()
            self.sin_close_windowsSetZero.emit()
            self.sin_close_windowsFeedback.emit()
            if self.__string_robot_Enabled:
                self.sin_stop_robot_operation.emit()
            event.accept()
        else:
            event.ignore()

    ###############  ｒｏｓ设置  #######################################

    def __ros_state_feedback_Set(self):
        if(self.action_6.isChecked()):
            self.__ros_feedback_flag = True
            # print self.__ros_feedback_flag
        else:
            self.__ros_feedback_flag = False
            # print self.__ros_feedback_flag

    ###############  检测需发送ｒｏｓ消息类型  end #######################################

    def __lineEdit_default_Set(self):
        # 笛卡尔空间控制界面　默认输入设置
        self.lineEdit_26.setText(str(586.4))    # X mm
        self.lineEdit_27.setText(str(0))        # Y
        self.lineEdit_28.setText(str(0))        # Z 
        self.lineEdit_29.setText(str(0))        # RX deg
        self.lineEdit_30.setText(str(0))        # RY
        self.lineEdit_31.setText(str(180))      # RZ

        self.lineEdit_34.setText(str(0))        # X mm/s
        self.lineEdit_37.setText(str(0))        # Y
        self.lineEdit_32.setText(str(0))        # Z 
        self.lineEdit_33.setText(str(0))        # RX deg/s
        self.lineEdit_36.setText(str(0))        # RY
        self.lineEdit_35.setText(str(0))        # RZ

        # 关节空间默认值
        self.lineEdit_7.setText(str(0))
        self.lineEdit_8.setText(str(0))
        self.lineEdit_9.setText(str(0))
        self.lineEdit_10.setText(str(0))
        self.lineEdit_11.setText(str(0))
        
        self.lineEdit_12.setText(str(5))

        self.lineEdit_23.setText(str(5))

# 新增的界面显示初始化
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

        # goal-base关机运行的位置命令
        self.lineEdit_59.setText(" ")
        self.lineEdit_65.setText(" ")
        self.lineEdit_68.setText(" ")
        self.lineEdit_69.setText(" ")
        self.lineEdit_70.setText(" ")

         # goal-base关机运行的速度命令
        self.lineEdit_71.setText(" ")
        self.lineEdit_73.setText(" ")
        self.lineEdit_74.setText(" ")
        self.lineEdit_75.setText(" ")
        self.lineEdit_76.setText(" ")

        # 转换后是否正确
        #self.lineEdit_72.setText(str(0))

    ############## 更新逆运动学基座 ####################################################
    def set_base_0(self):
        self.__base_flag = True
        self.radioButton_2.setChecked(False)
        self.sin_update_gripper_base.emit(self.__base_flag)

        pass

    def set_base_6(self):
        self.__base_flag = False
        self.radioButton.setChecked(False)
        self.sin_update_gripper_base.emit(self.__base_flag)
        pass
    ############## 更新逆运动学基座  end ###############################################


    # tab 页数改变
    def tab_change(self, data):
        pass

################## 接收数据界面 #######################################################        
# 新增函数
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


# 在界面展示目标位置到基坐标系下的位姿数据
    def show_goal_to_base_position_data(self, data):
        self.lineEdit_50.setText(str(data[0]))
        self.lineEdit_51.setText(str(data[1]))
        self.lineEdit_52.setText(str(data[2]))
        self.lineEdit_53.setText(str(data[3]))
        self.lineEdit_54.setText(str(data[4]))
        self.lineEdit_55.setText(str(data[5]))
        pass

# 在界面显示转换成功后关机电机位置指令
    def show_goal_to_base_motor_pos_command_data(self, data_pos, data_vel):
        self.lineEdit_59.setText(str(data_pos[0]))
        self.lineEdit_65.setText(str(data_pos[1]))
        self.lineEdit_68.setText(str(data_pos[2]))
        self.lineEdit_69.setText(str(data_pos[3]))
        self.lineEdit_70.setText(str(data_pos[4]))

        self.lineEdit_71.setText(str(data_vel[0]))
        self.lineEdit_73.setText(str(data_vel[1]))
        self.lineEdit_74.setText(str(data_vel[2]))
        self.lineEdit_75.setText(str(data_vel[3]))
        self.lineEdit_76.setText(str(data_vel[4]))

        self.goal_to_base_motor_temp_position_command[0] = data_pos[0]
        self.goal_to_base_motor_temp_position_command[1] = data_pos[1]
        self.goal_to_base_motor_temp_position_command[2] = data_pos[2]
        self.goal_to_base_motor_temp_position_command[3] = data_pos[3]
        self.goal_to_base_motor_temp_position_command[4] = data_pos[4]

        self.goal_to_base_motor_temp_velocity_command[0] = data_vel[0] * self.__direction_joints[0]
        self.goal_to_base_motor_temp_velocity_command[1] = data_vel[1] * self.__direction_joints[1]
        self.goal_to_base_motor_temp_velocity_command[2] = data_vel[2] * self.__direction_joints[2]
        self.goal_to_base_motor_temp_velocity_command[3] = data_vel[3] * self.__direction_joints[3]
        self.goal_to_base_motor_temp_velocity_command[4] = data_vel[4] * self.__direction_joints[4]

        for i in range(len(self.goal_to_base_motor_temp_position_command)):
            self.goal_to_base_motor_temp_position_command[i] = Modular_robot_control_func.__user_data_to_motor(self.goal_to_base_motor_temp_position_command[i],self.__zero_pos_joints[i],self.__direction_joints[i])

        pass

# 在界面显示转换成功后关机电机速度指令
    def show_goal_to_base_motor_vel_command_data(self, data):
        self.lineEdit_71.setText(str(data[0]))
        self.lineEdit_73.setText(str(data[1]))
        self.lineEdit_74.setText(str(data[2]))
        self.lineEdit_75.setText(str(data[3]))
        self.lineEdit_76.setText(str(data[4]))
        pass


    # 接收 管道数据(槽函数)
    def receive_data(self):


        if self.__robot_enabled_flag and self.__which_robot == 0:

            

            # 调用管道通信类线程
            self.relative_pose_process = Get_fifo_relative_position_thread()  

            # 相对位姿展示在Ui
            self.relative_pose_process.receive_relative__joint_position_data.connect(self._show_relative__joint_position_data)  

            # 储存相对位姿
            self.relative_pose_process.receive_relative__joint_position_data.connect(self._get_relative_position_data_)

            self.relative_pose_process.start()  # 线程启动
            pass
            


    # 转换 接收到的位姿到基座标的表示(槽函数)
    def translate_relative_position_to_base(self, data):
        # 反馈界面清0
        self.lineEdit_72.setText(" ")
        if self.__robot_enabled_flag and self.__which_robot == 0:
                             
            self.translate_pos_to_base_fun = Translate_goal_to_base_thread(self._get_relative_position_data, self.__which_robot, self.__base_flag, self.__pos_joints)
            self.translate_pos_to_base_fun.goal_to_base_position_.connect(self.show_goal_to_base_position_data)
            self.translate_pos_to_base_fun.wrong_translation_feedback.connect(self.show_determine_whether_translation_correct_flag)
            
            # 在ui界面显示转换后电机关机运动位置+速度指令
            self.translate_pos_to_base_fun.goal_to_base_motor_pos_command.connect(self.show_goal_to_base_motor_pos_command_data)
            
            
            self.translate_pos_to_base_fun.start()
                        
        pass
            
      

    # 运行 转换结果(槽函数)
    def run_translation_to_base(self):
        print "waiting...."
        if self.goal_to_base_tag_ == "True":
            # 发送关节位置和速度指令
            self.sin_joint_position.emit([self.goal_to_base_motor_temp_position_command, self.goal_to_base_motor_temp_velocity_command])
        else:
            print "run false...."
        pass


    # 在ui界面显示逆解转换成功与否   
    def show_determine_whether_translation_correct_flag(self, data):
        self.lineEdit_72.setText(str(data))
        self.goal_to_base_tag_ = str(data)
        pass

################## 接收数据界面 end #######################################################  