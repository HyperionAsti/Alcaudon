#include "hyperion_line_tracker_alg.h"

HyperionLineTrackerAlgorithm::HyperionLineTrackerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

HyperionLineTrackerAlgorithm::~HyperionLineTrackerAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void HyperionLineTrackerAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// HyperionLineTrackerAlgorithm Public API
void HyperionLineTrackerAlgorithm::pid_output(float &err, float &out, bool reset)
{
  static float prev_mean_err = 0;
  static float integral = 0;
  static float derivative = 0;

  if (reset)
  {
    integral = 0;
    derivative = 0;
    prev_mean_err = 0;
  }
  else
  {
    float mean_err = (prev_mean_err*this->config_.pid_mean_size + err)/(this->config_.pid_mean_size + 1); //~= the mean of the last pid_mean_size+1 errors
    derivative = (mean_err - prev_mean_err)*this->config_.pid_freq;
    integral += (mean_err/this->config_.pid_freq);
    out=this->config_.kp*mean_err + this->config_.ki*integral + this->config_.kd*derivative;
    prev_mean_err = mean_err;
  }

}

void HyperionLineTrackerAlgorithm::pid_tunning(float &err, float &out, float &kp, float &ki, float &kd, bool reset)
{
  static float prev_mean_err = 0;
  static float integral = 0;
  static float derivative = 0;

  if (reset)
  {
    integral = 0;
    derivative = 0;
    prev_mean_err = 0;
  }
  else
  {
    float mean_err = (prev_mean_err*this->config_.pid_mean_size + err)/(this->config_.pid_mean_size + 1); //~= the mean of the last pid_mean_size+1 errors
    derivative = (mean_err - prev_mean_err)*this->config_.pid_freq;
    integral += (mean_err/this->config_.pid_freq);
    out=kp*mean_err + ki*integral + kd*derivative;
    prev_mean_err = mean_err;
  }

}