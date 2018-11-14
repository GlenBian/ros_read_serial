
/*****************************************************************************
** Includes
*****************************************************************************/

#include "youibot_bms/youibot_bms.h"

namespace bms {

/**
 * @brief 构造函数.
 *
 */
YouibotBms::YouibotBms() :
    serial_id("/dev/bms"),
    baudrate(115200),
    is_connected_bms(false),
    is_reset_euler(false),
    shutdown_requested(false),
    verbose(false),
    batteries_number(16),
    msg_(new youibot_msgs::Battery())
{}

/**
 * @brief 析构函数.
 *
 */
YouibotBms::~YouibotBms()
{
    std::cout<<"[[** YouibotBms **]] : waiting for youibot bms node to finish "<<std::endl;
    serial_bms.close();
    is_connected_bms = false;
    shutdown_requested = true;
}
/**
 * @brief 初始化函数，定义话题的订阅和发布，参数管理器的配置和打开串口等.
 *
 * @param ros::NodeHandle
 *
 * @return 如果正确初始化则返回true，否则返回false
 */
bool YouibotBms::init(ros::NodeHandle &nh)
{
    // buffer初始化
    ask_buffer.clear();
    ask_buffer.push_back(0x0A); // 电池地址
    ask_buffer.push_back(0xB0); // 读命令
    ask_buffer.push_back(0x00); // 数据偏移地址
    ask_buffer.push_back(0x43); // 返回数据长度

    // 参数获取
    nh.param<std::string>("serial_id", serial_id , "/dev/bms");
    nh.param<int32_t>("baudrate" , baudrate , 9600);
    nh.param<bool>("verbose",verbose,true);
    nh.param<int>("batteries_number",batteries_number,16);
    // 话题发布
    bms_publisher = nh.advertise<youibot_msgs::Battery>("bms_data",100,true);
    // 打开串口
    try {
        //初始化串口
        const char* mobileName = serial_id.data();
        serial_bms.setPort(mobileName);
        // 设置波特率
        serial_bms.setBaudrate(baudrate);
        // 设置打开超时时间
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_bms.setTimeout(to);
        serial_bms.open();
        is_connected_bms = true;
        os.str("");
        os << "**bms** device is connected.";
        ROS_INFO_STREAM(os.str());
    }
    catch (const serial::IOException &e)
    {
        os.str("");
        os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
        ROS_WARN_STREAM(os.str());
        is_connected_bms = false;
    }
    catch (const serial::SerialException &e)
    {
        os.str("");
        os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
        ROS_WARN_STREAM(os.str());
        is_connected_bms = false;
    }
    catch (const serial::PortNotOpenedException &e)
    {
        os.str("");
        os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
        ROS_WARN_STREAM(os.str());
        is_connected_bms = false;
    }
    // 串口监控线程
    thread.start(&YouibotBms::serial_manager,*this);

    return true;
}
/**
 * @brief 循环调用的函数.
 *
 * @return 如果正确初始化则返回true，否则返回false
 */
void YouibotBms::serial_manager()
{
    while (!shutdown_requested) {
        if(!is_connected_bms)
        {
            try {
                //初始化串口
                const char* mobileName = serial_id.data();
                serial_bms.setPort(mobileName);
                // 设置波特率
                serial_bms.setBaudrate(baudrate);
                // 设置打开超时时间
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                serial_bms.setTimeout(to);
                serial_bms.open();
                os.str("");
                os << "**bms** device is connected.";
                ROS_INFO_STREAM(os.str());
                is_connected_bms = true;
            }
            catch (const serial::IOException &e)
            {
                os.str("");
                os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
                ROS_WARN_STREAM(os.str());
                sleep(3); // 每5s尝试打开一次串口，如果打开串口失败的话
                is_connected_bms = false;
                continue;
            }
            catch (const serial::SerialException &e)
            {
                os.str("");
                os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
                ROS_WARN_STREAM(os.str());
                sleep(3); // 每5s尝试打开一次串口，如果打开串口失败的话
                is_connected_bms = false;
                continue;
            }
            catch (const serial::PortNotOpenedException &e)
            {
                os.str("");
                os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
                ROS_WARN_STREAM(os.str());
                sleep(3); // 每5s尝试打开一次串口，如果打开串口失败的话
                is_connected_bms = false;
                continue;
            }
        }
    }
}

/**
 * @brief 循环调用的函数.
 *
 * @return 如果正确初始化则返回true，否则返回false
 */
bool YouibotBms::update()
{
    int n = 0;
//    std::cout << is_connected_bms << " " << is_reset_euler << endl;
    if(is_connected_bms)
    {
        // 进入正常的读取程序
        try
        {
            n = serial_bms.write(ask_buffer);
        }
        catch(const serial::SerialException &e)
        {
            os.str("");
            os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
            ROS_WARN_STREAM(os.str());
            is_connected_bms = false;
            return false;
        }
        catch(const serial::IOException &e)
        {
            os.str("");
            os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
            ROS_WARN_STREAM(os.str());
            is_connected_bms = false;
            is_reset_euler = false;
            return false;
        }

        try
        {
            buffer.clear();
            n = serial_bms.read(buffer,67);
        }
        catch(const serial::SerialException &e)
        {
            os.str("");
            os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
            ROS_WARN_STREAM(os.str());
            is_connected_bms = false;
            return false;
        }
        catch(const serial::IOException &e)
        {
            os.str("");
            os << "**bms** device failed to open, waiting... [" + std::string(e.what()) + "]";
            ROS_WARN_STREAM(os.str());
            is_connected_bms = false;
            return false;
        }
        if(verbose)
        {
            printf("rece: ");
            for(int i = 0 ; i < n ; i++)
            {
                printf("%x ",buffer[i]);
            }
            printf("\r\n");
        }
        msg_.reset(new youibot_msgs::Battery());
        msg_->header.stamp = ros::Time::now();
        // 读取电芯电压
        for(unsigned int i = 0 ; i < batteries_number ; i++)
        {
            msg_->vcells.push_back(buffer[2*i] * 256 + buffer[2*i+1]);
        }
        msg_->voltage = buffer[32] * 256 * 256 * 256 + buffer[33] * 256 * 256 + buffer[34] * 256 + buffer[35];
        msg_->curcadc = buffer[36] * 256 * 256 * 256 + buffer[37] * 256 * 256 + buffer[38] * 256 + buffer[39];
        if((buffer[40] * 0x80) == 0x80)
        {
            msg_->temperatures.push_back(-((buffer[40] & 0x7F) * 256 + buffer[41]));
        }else
        {
            msg_->temperatures.push_back(buffer[40] * 256 + buffer[41]);
        }
        if((buffer[42] * 0x80) == 0x80)
        {
            msg_->temperatures.push_back(-((buffer[42] & 0x7F) * 256 + buffer[43]));
        }else
        {
            msg_->temperatures.push_back(buffer[42] * 256 + buffer[43]);
        }
        if((buffer[44] * 0x80) == 0x80)
        {
            msg_->temperatures.push_back(-((buffer[44] & 0x7F) * 256 + buffer[45]));
        }else
        {
            msg_->temperatures.push_back(buffer[44] * 256 + buffer[45]);
        }
        msg_->fcc = buffer[46] * 256 * 256 * 256 + buffer[47] * 256 * 256 + buffer[48] * 256 + buffer[49];
        msg_->rc = buffer[50] * 256 * 256 * 256 + buffer[51] * 256 * 256 + buffer[52] * 256 + buffer[53];
        msg_->rsoc = buffer[54] * 256 + buffer[55];

        if ((buffer[59] & 0x80) == 0x80)
        {
            msg_->ischarging = 1;
        }
        else
        {
            msg_->ischarging = 0;
        }
        if ((buffer[59] & 0x40) == 0x40)
        {
            msg_->isdischarging = 1;
        }
        else
        {
            msg_->isdischarging = 0;
        }
        bms_publisher.publish(msg_);
        return true;
    }else
    {
        ROS_INFO("**bms** device failed to open, waiting...");
        return false;
    }
}

}
