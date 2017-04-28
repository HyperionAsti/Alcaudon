#ifndef _TIMEOUT_H
#define _TIMEOUT_H

#include <ros/ros.h>

class CROSTimeout
{
  private:
    bool active;
    ros::Time start_time;
    ros::Duration time_period;
    // mutex
    pthread_mutex_t access_;
  public:
    CROSTimeout();
    void start(const ros::Duration &time);
    void stop(void);
    bool timed_out(void);
    ~CROSTimeout();
};

#endif
