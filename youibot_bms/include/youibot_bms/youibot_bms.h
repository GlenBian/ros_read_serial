#ifndef YOUIBOT_BMS_DATA_H__
#define YOUIBOT_BMS_DATA_H__

/*****************************************************************************
** Includes
*****************************************************************************/

/*********************
 ** system
 **********************/
#include <string>
#include <iostream>
#include <stdint.h>
#include <vector>
/*********************
 ** youibot_serial
 **********************/
#include <youibot_serial/serial.h>
#include <youibot_serial/v8stdint.h>
/*********************
 ** youibot_math_toolkit
 **********************/
#include <youibot_math_toolkit/common.hpp>
#include <youibot_math_toolkit/geometry.hpp>
/*********************
 ** ros
 **********************/
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
/*********************
 ** youibot_msgs
 **********************/
#include <youibot_msgs/EulerAngle.h>
#include <youibot_msgs/Battery.h>
/*********************
 ** ecl
 **********************/
#include <ecl/threads.hpp>

#define PI 3.1415926

using namespace std;

/*********************
 ** namespace
 **********************/
namespace bms {

/*********************
 ** class
 **********************/

class YouibotBms
{
public:
    YouibotBms();
    ~YouibotBms();

    /*********************
     ** Configuration
     **********************/
    bool init(ros::NodeHandle& nh);
    /*********************
     ** Update
     **********************/
    bool update();
    /*********************
     ** Serial Manager
     **********************/
    void serial_manager();

public:
    /*********************
    ** Thread
    **********************/
    ecl::Thread thread;
    bool shutdown_requested; // helper to shutdown the worker thread.
    // 存放读取命令的buffer
    vector<unsigned char> ask_buffer;
    // 存放设备->PC的数据容器
    std::vector<unsigned char> buffer;
    // 串口名称
    string serial_id;
    // 串口波特率
    int32_t baudrate;
    // 电池的额定电压
    int8_t standard_voltage;
    // 电芯数量
    int batteries_number;
    // 串口接口
    serial::Serial serial_bms;
    // 串口是否打开
    bool is_connected_bms;
    // 欧拉角是否清零
    bool is_reset_euler;
    // 是否显示串口数据
    bool verbose;
    // 话题发布
    ros::Publisher bms_publisher;
    ostringstream os;
    // bms 消息类型
    youibot_msgs::BatteryPtr msg_;
};

}

#endif
