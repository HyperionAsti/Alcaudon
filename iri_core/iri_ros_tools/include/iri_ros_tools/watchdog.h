#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <ros/ros.h>

class CROSWatchdog
{
  private:
    ros::Time start_time;
    ros::Duration max_time;
    pthread_mutex_t access_;
  public:
    CROSWatchdog();
    void reset(ros::Duration time);
    bool is_active(void);
    ~CROSWatchdog();
};

#endif
