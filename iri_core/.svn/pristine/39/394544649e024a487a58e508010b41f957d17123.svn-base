#include "iri_ros_tools/timeout.h"

CROSTimeout::CROSTimeout()
{
  // initialize mutex
  pthread_mutex_init(&this->access_,NULL);
  this->time_period=ros::Duration(0);
  this->start_time=ros::Time::now();
  this->active=false;
}

void CROSTimeout::start(const ros::Duration &time)
{
  pthread_mutex_lock(&this->access_);
  this->time_period = time;
  this->start_time=ros::Time::now();
  this->active=true;
  pthread_mutex_unlock(&this->access_);
}

void CROSTimeout::stop(void)
{
  pthread_mutex_lock(&this->access_);
  this->active=false;
  pthread_mutex_unlock(&this->access_);
}

bool CROSTimeout::timed_out(void)
{
  ros::Time new_time=ros::Time::now();

  if(this->active)
  {
    pthread_mutex_lock(&this->access_);
    if((new_time-this->start_time)>this->time_period)
    {
      pthread_mutex_unlock(&this->access_);
      this->active=false;
      return true;
    }
    else
    {
      pthread_mutex_unlock(&this->access_);
      return false;
    }
  }
}

CROSTimeout::~CROSTimeout()
{
  // destray mutex
  pthread_mutex_destroy(&this->access_);
}

