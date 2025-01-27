#include <ros/ros.h>
#include <std_msgs/UInt8.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <signal.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB_tof"
//#define TOF_MULTI

ros::Publisher touch_pub;
// ros::Publisher tof_marker_pub;

struct termios tio;                 // シリアル通信設定
int fd;
int baudRate = B115200;
char receive_buff[512];
unsigned char touch[8];
unsigned long frame;
char hd[16];
// int tof_limit;
// int tof_continuous_count;
// int tof_duration_count;
bool touch_verbose = true;

// int tof_cont[4] = {0};
bool loop = true;

void signal_handler(int signum)
{
  puts("SHUTDONW NOW");
  close(fd);
  loop = false;
  ros::shutdown(); // ノードの停止。これを呼ぶとspinから抜ける。
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "touch_node");
  ros::NodeHandle nh;
  touch_pub = nh.advertise<std_msgs::UInt8>("/touch", 1);
  // tof_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/tof_marker", 1);

  // nh.getParam("/mechanum_control/tof_limit", tof_limit);
  // nh.getParam("/mechanum_control/tof_continuous_count", tof_continuous_count);
  // nh.getParam("/mechanum_control/tof_duration_count", tof_duration_count);
  // nh.getParam("/mechanum_control/tof_verbose", tof_verbose);
  // ROS_INFO("tof_limit: %d", tof_limit);
  // ROS_INFO("tof_continuous_count: %d", tof_continuous_count);
  // ROS_INFO("tof_duration_count: %d", tof_duration_count);

  // int tof_cliff_cc = 0;

  signal(SIGINT, signal_handler);

  while(loop){
    fd = open(SERIAL_PORT, O_RDWR | O_NONBLOCK);     // デバイスをオープンする
    if (fd < 0) {
        ROS_ERROR("open error");
        return false;
    }

    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );
    cfmakeraw(&tio);                    // RAWモード
    tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う
    ioctl(fd, TCSETS, &tio);            // ポートの設定を有効にする

    int touch_stage = 0;
    int pos;
    int wait_count = 0;
    int lenzero_count = 0;
    bool send_disp = false;
    ros::Rate rate(100);  //Hz
    while(ros::ok()){
      int len = read(fd, receive_buff, 512);
      if(len < -1){
        ROS_ERROR("tty error. %d\n", len);
        return 0;
      }else{
        if(len > 0){
          receive_buff[len] = 0;
          if (touch_verbose) ROS_INFO("receive data : %s", receive_buff);
          std::string str = receive_buff;
          if(str.find("Blue Pill") != std::string::npos){
            touch_stage = 1;
          }else if(str.find("TOUCH") == 0){
            hd[0] = 0;
            sscanf(receive_buff, "TOUCH,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu", &touch[0], &touch[1], &touch[2], &touch[3], &touch[4], &touch[5], &touch[6], &touch[7]);
            if(touch_verbose)
              ROS_INFO("touch %s %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu\n", hd, touch[0], touch[1], touch[2], touch[3], touch[4], touch[5], touch[6], touch[7]);
            std_msgs::UInt8 touch_msg;
            uint8_t t = 0x01;
            uint8_t d = 0;
            for(int i = 0; i < 8; i++){
              if(touch[i] > 0) d |= t;
              t = t << 1;
            }
            touch_msg.data = d;
            touch_pub.publish(touch_msg);
          }
          wait_count = 0;
        }else if(len == 0 || len == -1){
          lenzero_count++;
          if(lenzero_count > 100 && !send_disp){
            std::string cmd = "disp\r";
            write(fd, cmd.c_str(), cmd.length());
            send_disp = true;
            lenzero_count = 0;
          }else if(lenzero_count > 1000 && send_disp){
            ROS_ERROR("no response.");
            break;
          }
        }else{
        }

      }
      ros::spinOnce();
      rate.sleep();
    }
    close(fd);
    ros::shutdown();
  }
  return 0;
}
