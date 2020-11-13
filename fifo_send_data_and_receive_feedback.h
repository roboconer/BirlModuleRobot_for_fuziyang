#ifndef __FIFO_SEND_DATA_AND_RECEIVE_FEEDBACK_H__
#define __FIFO_SEND_DATA_AND_RECEIVE_FEEDBACK_H__


//  extern double final_pose[6];  //位姿数据缓存(source: location_matrix_handle.cpp)
#define _PATH_NAME_WRITE_ "/tmp/file.in"   // 发送数据的管道
#define _PATH_NAME_READ_  "/tmp/file.out"  // 接收反馈数据的管道

char feedback_wrong_tag[1] = "";  // 接收的反馈标志位（当收到字符'1' == 数据有错; 当收到字符'0' == 数据逆解成功）

bool send_relative_position(double*);
bool feedback_translation_wrong_tag(void);



#endif