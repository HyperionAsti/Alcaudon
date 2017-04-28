#include "iri_ros_tools/watchdog.h"

CROSWatchdog::CROSWatchdog()
{
  pthread_mutex_init(&this->access_,NULL);
  this->max_time=ros::Duration(0);
}

void CROSWatchdog::reset(ros::Duration time)
{
  pthread_mutex_lock(&this->access_);
  this->max_time=time;
  pthread_mutex_unlock(&this->access_);
}

bool CROSWatchdog::is_active(void)
{
  this->start_time=ros::Time::now();
  ros::Time current_time=ros::Time::now();

  pthread_mutex_lock(&this->access_);
  this->max_time-=(current_time-this->start_time);
  this->start_time=current_time;
  if(this->max_time.toSec()<=0.0)
  {
    pthread_mutex_unlock(&this->access_);
    return true;
  }
  else
  {
    pthread_mutex_unlock(&this->access_);
    return false;
  }	
}

CROSWatchdog::~CROSWatchdog()
{
  pthread_mutex_destroy(&this->access_);
}

