/*****************************************************************************
 *        规划命令代码与控制命令代码的控制信息管道传输
 *        GDUT BIRL 2020
 *        Version number :  0.10
 *        Author:           KIKO
 *        Date:             2020-11
 *****************************************************************************/
/*
  function: send_relative_position(double* final_pose) 

            final_pose 为发送的目标位置到相机位置的相对位姿( xyzrpy, unit: mm /degree)


  function: feedback_translation_wrong_tag()
  
            feedback_wrong_tag : 接收的反馈标志位（当收到字符'1' == 数据有错; 当收到字符'0' == 数据逆解成功）
*/

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fifo_send_data_and_receive_feedback.h>



//  extern double final_pose[6];  //位姿数据缓存(source: location_matrix_handle.cpp)


bool send_relative_position(double* final_pose) {  
      int ret = 0;

    /* 先删除之前可能遗留的管道文件，然后再次创建它 */
	unlink( _PATH_NAME_WRITE_ );

    int res_write = mkfifo(_PATH_NAME_WRITE_ , S_IFIFO|0666);  // //创建一个命名管道且存取权限为0666，即创建者、与创建者同组的用户、其他用户对该命名管道的访问权限都是可读可写
    
    char final_pose_[6][8] = {"","","","","",""};  // 发送数据的初始化(开辟一个字符数组来将double转化为char存储和发送)
    memset(final_pose_,'\0',sizeof(final_pose_));
    
    int fd_write = open(_PATH_NAME_WRITE_ , O_WRONLY);   //以只写方式打开管道

    for(int i = 0; i<6; i++) {
      sprintf(final_pose_[i], "%lf", final_pose[i]);    // 将double数组转化为字符数组
      ret = write(fd_write, final_pose_[i], sizeof(final_pose_[i]));  //写入数据         
    }
    if(ret < 0)    printf("Fail to write.\n"); //写入失败的话
    
    close(fd_write);                           // 关闭写入管道
    return 0;
 }
 
                   


 bool feedback_translation_wrong_tag() {

      int res = 0;

      int fd_read = open(_PATH_NAME_READ_ , O_RDONLY);  //以只读方式打开管道

      if (fd_read < 0) {
          printf("open res_read_fifo fail");
      }
      else {
        res = read(fd_read, feedback_wrong_tag, 1024);  //这里注意feedback_wrong_tag需要一个指针的取值符（read/write函数的原型要求）
       // 如果不存在此FIFO或管道已经打开写端时，read返回0；
        if (res > 0 && feedback_wrong_tag != "\0") {

            if (feedback_wrong_tag[0] == '0') { 
                printf("feedback_wrong_tag: %s \n",feedback_wrong_tag);    // 接收到字符'0',机器人逆解成功
                close(fd_read);
                return true;
            }

            else if (feedback_wrong_tag[0] == '1') {
                printf("feedback_wrong_tag: %s \n", feedback_wrong_tag);   // 接收到字符'1',机器人逆解失败
                close(fd_read);
                return false;
            }
            
            else {
            printf("something wrong while reading fifo.\n");                  // 读取管道出错
            close(fd_read);
            return false;
            }
        }

      else {
            printf("Fail to read feedback_wrong_tag.\n");                  // 读取管道出错
            close(fd_read);
            return false;
        }
    }
 }


