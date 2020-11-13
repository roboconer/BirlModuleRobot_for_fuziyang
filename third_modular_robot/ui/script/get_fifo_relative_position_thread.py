#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@version: python2.7
@author:
@contact: 
@software: RoboWareStudio
@file: get_fifo_relative_position_thread.py

@biref:　通过命名管道获取相对位置数据

@note: 如果会出现找不到管道路径，可是试试看是不是.vscode的setting.json为c++的，而没有python的配置
"""

#  在read/write fifo 时，需要先刷新缓存区(`http://www.cocoachina.com/cms/wap.php?action=article&id=79893)
# python中，以双下划线"__"开头的函数和变量，是作为该类成员的私有成员，不能进行外部调用(https://www.cnblogs.com/navysummer/p/10438779.html)
from PyQt5.QtCore import QThread,pyqtSignal,QMutex,QObject
from time import sleep
import os, time, sys, multiprocessing,threading
from get_positive_solution_thread import Get_positive_solution_thread
sys.path.append('~/ros/BirlModuleRobot/src/third_modular_robot/birl_modular_robot/script/') 
from inverse_solution_client import Inverse_solution_client
from positive_solution_client import Positive_solution_client

#from modular_robot_control_func import Modular_robot_control_func 





_PATH_NAME_READ_  = "/tmp/file.in"
_PATH_NAME_WRITE_ = "/tmp/file.out"

class Get_fifo_relative_position_thread(QThread):

    global relative_pose
    global res_read

    relative_pose = [0,0,0,0,0,0]  # 初始化存储数组
    res_read = 0
    # 接受相对关节位姿数据(单位： mm / degree)
    receive_relative__joint_position_data  = pyqtSignal(list)

    # 错误转换反馈
    #wrong_translation_feedback = pyqtSignal()

    def __init__(self):
        super(Get_fifo_relative_position_thread, self).__init__()
        #self.feedback_translation_tag = feedback_tag
        pass
          
    
    def run(self):
        
        self.get_fifo_relative_position()

        pass


    # 通过管道获取相对位姿
    def get_fifo_relative_position(self):

        #if os.path.exists(_PATH_NAME_READ_):  # 重新启动管道
        #    os.remove(_PATH_NAME_READ_)

        res_read = os.open(_PATH_NAME_READ_, os.O_RDONLY)  # 

        if res_read < 0:
            print "open res_read_fifo fail"
        else:
            num = 0
            while (num < 6):
                print "111111"
                cache_data = os.read(res_read, 8)  #缓冲区长度(8位字符)
                relative_pose[num] = float(cache_data)  # 将缓冲区字符串转成 float
                print "received the relative_position in fifo: %f" % relative_pose[num]
                num = num + 1
            
                if len(relative_pose) == 0:
                    print "The fifo now without any data."
                    break            
            self.receive_relative__joint_position_data.emit(relative_pose)
            return relative_pose 
        sleep(0.1)    
        os.close(res_read)



class Translate_goal_to_base_thread(QThread):

    goal_to_base_position_ = pyqtSignal(list)

    # 错误转换反馈
    wrong_translation_feedback = pyqtSignal(str)

    # 逆解后机器人的关节位置+速度命令
    goal_to_base_motor_pos_command = pyqtSignal(list, list)


    def __init__(self, _get_relative_position_data, __which_robot, __base_flag, __pos_joints):
        super(Translate_goal_to_base_thread, self).__init__()
        self._now_robot_pasitive_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._goal_to_base_position_data  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.descartes_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.__goal_to_base_position_data_meter = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._get_relative_position_data = _get_relative_position_data
        self.__which_robot = __which_robot
        self.__base_flag = __base_flag
        self.__pos_joints = __pos_joints
         # 线程同步锁
        self.__mutex = QMutex()
        # m 转为 mm
        self.__scale = 1000

    def run(self):
        print "213"

        self.get_now_positive_fun()

        pass

    # 获得机器人当前末端的位姿
    def get_now_positive_data(self, data):
        #  (xyzwpr) unit:(meter,degree)
        self._now_robot_pasitive_position[0] = data[0]
        self._now_robot_pasitive_position[1] = data[1]
        self._now_robot_pasitive_position[2] = data[2]
        self._now_robot_pasitive_position[3] = data[3]
        self._now_robot_pasitive_position[4] = data[4]
        self._now_robot_pasitive_position[5] = data[5]
        
        # 目标点到基座标的位姿表示   (xyzwpr) unit:( mm, degree)
        self._goal_to_base_position_data[0] = self._get_relative_position_data[0] + self._now_robot_pasitive_position[0] * self.__scale # goal to base: X (将 m 转化为 mm)
        self._goal_to_base_position_data[1] = self._get_relative_position_data[1] + self._now_robot_pasitive_position[1] * self.__scale # goal to base: Y (将 m 转化为 mm)
        self._goal_to_base_position_data[2] = self._get_relative_position_data[2] + self._now_robot_pasitive_position[2] * self.__scale # goal to base: Z (将 m 转化为 mm)
        self._goal_to_base_position_data[3] = self._get_relative_position_data[3] + self._now_robot_pasitive_position[3] # goal to base: RX 
        self._goal_to_base_position_data[4] = self._get_relative_position_data[4] + self._now_robot_pasitive_position[4] # goal to base: RY
        self._goal_to_base_position_data[5] = self._get_relative_position_data[5] + self._now_robot_pasitive_position[5] # goal to base: RZ
        
        
        self.goal_to_base_position_.emit(self._goal_to_base_position_data)
        
        # 发送给逆解公式(xyzwpr) unit:( m, degree)
        self.__goal_to_base_position_data_meter[0] = self._goal_to_base_position_data[0]/1000
        self.__goal_to_base_position_data_meter[1] = self._goal_to_base_position_data[1]/1000
        self.__goal_to_base_position_data_meter[2] = self._goal_to_base_position_data[2]/1000
        self.__goal_to_base_position_data_meter[3] = self._goal_to_base_position_data[3]
        self.__goal_to_base_position_data_meter[4] = self._goal_to_base_position_data[4]
        self.__goal_to_base_position_data_meter[5] = self._goal_to_base_position_data[5]
        pass

     # 通过管道传送转换标志反馈
    def feedback_translation_tag_fun(self, feedback_translation_tag):

        print "mkfifo:1334"
        if os.path.exists(_PATH_NAME_WRITE_):
            os.remove(_PATH_NAME_WRITE_)
            
        os.mkfifo(_PATH_NAME_WRITE_)
        print "mkfifo:1"
        res_write = os.open(_PATH_NAME_WRITE_, os.O_WRONLY)
        print "mkfifo:0"
        if res_write < 0:
            print "open res_write_feedback_tag_fifo wrong"
        else:    
            # ret = os.write(res_write, self.feedback_translation_tag)
            ret = os.write(res_write, str(feedback_translation_tag))


            print "send the feedback_translation_tag to partner..."
            if ret < 0:
                print "send error,please try argin..."
            else:
                # print "send sucessful!,tag is: %s" % self.feedback_translation_tag
                print "send sucessful!,tag is: %s" % str(feedback_translation_tag)
           
        os.close(res_write)
        pass

        
    # 获得机器人当前末端的位姿    
    def get_now_positive_fun(self):    
        print "213"
        
        # 调用机器人正解函数，获得机器人当前的位姿
        self.descartes_position = Positive_solution_client(self.__which_robot, self.__base_flag,  self.__pos_joints)
        self.get_now_positive_data(self.descartes_position)

        print "213"
        
        # 调用机器人逆解函数，获得关节运动情况(含逆解正确与否的标志位)  
        [_joint_position_command, _joint_velocity_command, inverse_solution_tag] = Inverse_solution_client(1,  self.__base_flag, self.__goal_to_base_position_data_meter, [0.004, 0.002, 0.002, 0.002, 0.004], self.__pos_joints)
        sleep(0.5)
        print "123"


        # 发送关机运动和速度指令
        self.goal_to_base_motor_pos_command.emit(_joint_position_command, _joint_velocity_command)

        
        # 在ui界面显示逆解转换成功与否
        self.wrong_translation_feedback.emit(str(inverse_solution_tag))

        

        if str(inverse_solution_tag) == "False":
            feedback_translation_tag = 1
            

        elif str(inverse_solution_tag) == "True" or str(inverse_solution_tag) == "true":
            feedback_translation_tag = 0

        else:
            print "before send to fifo, Something wrong"

        #####################################
        # 通过管道传送转换标志反馈
        print "mkfifo:1334"
        if os.path.exists(_PATH_NAME_WRITE_):
            os.remove(_PATH_NAME_WRITE_)
            
        os.mkfifo(_PATH_NAME_WRITE_)
        print "mkfifo:1"
        res_write = os.open(_PATH_NAME_WRITE_, os.O_WRONLY)
        print "mkfifo:0"
        if res_write < 0:
            print "open res_write_feedback_tag_fifo wrong"
        else:    
            # ret = os.write(res_write, self.feedback_translation_tag)
            ret = os.write(res_write, str(feedback_translation_tag))


            print "send the feedback_translation_tag to partner..."
            if ret < 0:
                print "send error,please try argin..."
            else:
                # print "send sucessful!,tag is: %s" % self.feedback_translation_tag
                print "send sucessful!,tag is: %s" % str(feedback_translation_tag)
           
        os.close(res_write)

        ############################################

    
        pass


    













# if __name__ == "__main__":

#     while(1):
#         all_relative_pos = __get_fifo_relative_position()   # 接收对方的相对位姿数据
#         feedback_translation_tag = "1"   # "1"为有错误, "0"为无错  
#         __feedback_translation_tag(feedback_translation_tag)

"""



global relative_pose
global res_read

relative_pose = [0,0,0,0,0,0]  # 初始化存储数组
res_read = 0

sys.path.append("/tmp/")

_PATH_NAME_READ_  = "/tmp/file.in"
_PATH_NAME_WRITE_ = "/tmp/file.out"

_SIZE_TRANSFORM_ = 6  # 管道一次性传输的数组长度


def __get_fifo_relative_position():


    res_read = os.open(_PATH_NAME_READ_, os.O_RDONLY)  # 

    if res_read < 0:
        print "open res_read_fifo fail"
    else:
        num = 0
        while (num < _SIZE_TRANSFORM_):
            cache_data = os.read(res_read, 8)  #缓冲区长度(8位字符--char数组)
            relative_pose[num] = float(cache_data)  # 将缓冲区字符串转成 float
            print "received the relative_position in fifo: %f" % relative_pose[num]
            num = num + 1

            if len(relative_pose) == 0:
                print "The fifo now without any data."
                time.sleep(1)
                break

        return relative_pose 
            # self.receive_relative__joint_position_data.emit(relative_pose)
    time.sleep(1)
        
    os.close(res_read)

    # 通过管道传送转换标志反馈
def __feedback_translation_tag(feedback_translation_tag):

    if os.path.exists(_PATH_NAME_WRITE_):
        os.remove(_PATH_NAME_WRITE_)
        
    os.mkfifo(_PATH_NAME_WRITE_)

    res_write = os.open(_PATH_NAME_WRITE_, os.O_WRONLY)
    if res_write < 0:
        print "open res_write_feedback_tag_fifo wrong"
    else:    
        ret = os.write(res_write, feedback_translation_tag)

        print "send the feedback_translation_tag to partner..."
        if ret < 0:
            print "send error,please try argin..."
        else:
            print "send sucessful!,tag is: %s" % feedback_translation_tag
            
    os.close(res_write)





"""



