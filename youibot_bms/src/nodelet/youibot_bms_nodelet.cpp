
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "youibot_bms/youibot_bms.h"

namespace bms
{

class YouibotBmsNodelet : public nodelet::Nodelet
{
public:
  YouibotBmsNodelet() : shutdown_requested_(false) {}
  ~YouibotBmsNodelet()
  {
    shutdown_requested_ = true;
    update_thread_.join();
    printf("[[* YouibotBmsNodelet *]] : waiting for update thread to finish.\r\n");
  }
  virtual void onInit()
  {
    NODELET_DEBUG_STREAM("YouibotBmsNodelet : initialising nodelet...");
    youibot_bms_.reset(new YouibotBms());
    this->getPrivateNodeHandle().param<int>("major_frequency",frequency,50);
    // if there are latency issues with callbacks, we might want to move to process callbacks in multiple threads (use MTPrivateNodeHandle)
    if (youibot_bms_->init(this->getPrivateNodeHandle()))
    {
      update_thread_.start(&YouibotBmsNodelet::update, *this);
      NODELET_INFO_STREAM("YouibotBmsNodelet : initialised.");
    }
    else
    {
      NODELET_ERROR_STREAM("YouibotBmsNodelet : could not initialise! Please restart.");
    }
  }
private:
  void update()
  {
    ros::Rate spin_rate(frequency);
    while (!shutdown_requested_ && ros::ok())
    {   if(!youibot_bms_->update())
        {
            ROS_WARN("youibot_bms->update() return false!");
        }
        spin_rate.sleep();
    }
  }
  boost::shared_ptr<YouibotBms> youibot_bms_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
  int frequency;
};

} // namespace
PLUGINLIB_EXPORT_CLASS(bms::YouibotBmsNodelet, nodelet::Nodelet)
