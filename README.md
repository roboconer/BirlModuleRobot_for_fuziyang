# BirlModuleRobot_for_fuziyang
This is the personal UI control for fuziyang control the ModuleRobot.
## 本代码的实现功能：通过linux命名管道fifo传输控制指令给机器人控制界面，实现两个不同功能的界面的交互.

### 1、新建一个catkin/src工作空间，将third_modular_robot文件放入src文件夹下，执行catkin_make.

### 2、对相机捕获那端的source code,在需要进行管道通信的地方，调用fifo_send_data_and_receive_feedback.cpp.

### 3、启动机器人操作的ui界面，如third_modular_robot/README.md所提示.

### 4、操作机器人操作的ui界面，如third_modular_robot/manual/manual.pdf所提示.
