#include "hyperion_line_tracker_alg_node.h"

HyperionLineTrackerAlgNode::HyperionLineTrackerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HyperionLineTrackerAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
  this->loop_rate_ = 100;
  // [init publishers]
  this->speeds_publisher_ = this->public_node_handle_.advertise<hyperion_motor_driver::Speeds>("speeds", 1);
  
  // [init subscribers]
  this->joy_subscriber_ = this->public_node_handle_.subscribe("joy", 1, &HyperionLineTrackerAlgNode::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);

  this->ir_error_subscriber_ = this->public_node_handle_.subscribe("ir_error", 1, &HyperionLineTrackerAlgNode::ir_error_callback, this);
  pthread_mutex_init(&this->ir_error_mutex_,NULL);
  this->error = 0;
  this->stop = true;
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

HyperionLineTrackerAlgNode::~HyperionLineTrackerAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->joy_mutex_);
  pthread_mutex_destroy(&this->ir_error_mutex_);
  this->error = 0;
  this->stop = true;
}

void HyperionLineTrackerAlgNode::mainNodeThread(void)
{
  this->alg_.lock(); 
  static bool first = true;
  if (first)
  {
    this->loop_rate_ = this->config_.pid_freq;
    this->tunning_kp = this->config_.kp;
    this->tunning_ki = this->config_.ki;
    this->tunning_kd = this->config_.kd;
    first = false;
  }
  float pid_out = 0;
  
  switch (this->config_.pid_tunning)
  {
    case 0://none
      this->alg_.pid_output(this->error, pid_out, this->stop);
    break;
    default:
      this->alg_.pid_tunning(this->error, pid_out, this->tunning_kp, this->tunning_ki, this->tunning_kd, this->stop);
    break;
  }
  //ROS_INFO_STREAM("pid_out " << pid_out);
  if (this->stop != true)
  {
    this->speeds_msg_.right_speed = (pid_out > 0 ? (this->config_.max_speed - pid_out < this->config_.turn_speed_saturation ? this->config_.turn_speed_saturation : (int)(this->config_.max_speed - pid_out)) : this->config_.max_speed);
    this->speeds_msg_.left_speed = (pid_out > 0 ? this->config_.max_speed : (this->config_.max_speed + pid_out < this->config_.turn_speed_saturation ? this->config_.turn_speed_saturation : (int) (this->config_.max_speed + pid_out)));
  }
  else
  {
    this->speeds_msg_.right_speed = 0;
    this->speeds_msg_.left_speed = 0;
  }
  this->alg_.unlock();
  this->speeds_publisher_.publish(this->speeds_msg_);
  // [fill msg structures]
  // Initialize the topic message structure
  //this->speeds_Speeds_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->speeds_publisher_.publish(this->speeds_Speeds_msg_);

}

/*  [subscriber callbacks] */
void HyperionLineTrackerAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  ROS_DEBUG("HyperionLineTrackerAlgNode::joy_callback: New Message Received");

  this->alg_.lock();
  static bool button_pressed = false;
  if (msg->buttons[this->config_.stop_button] == 1 && button_pressed == false)
  {
    button_pressed = true;
    this->stop = !(this->stop);

    if (this->stop)
    {
      switch (this->config_.pid_tunning)
      {
        case 1://kp
          this->tunning_kp += this->config_.tunning_inc;
        break;    
        case 2://ki
          this->tunning_ki += this->config_.tunning_inc;
        break; 
        case 3://kd
          this->tunning_kd += this->config_.tunning_inc;
        break;   
      }
    }
    this->alg_.unlock();
    //ROS_INFO_STREAM("stop = " << this->stop);
  }
  else if (msg->buttons[this->config_.stop_button] == 0 && button_pressed == true)
  {
    button_pressed = false;
    this->alg_.unlock();
  }
  else
    this->alg_.unlock();
}

void HyperionLineTrackerAlgNode::joy_mutex_enter(void)
{
  pthread_mutex_lock(&this->joy_mutex_);
}

void HyperionLineTrackerAlgNode::joy_mutex_exit(void)
{
  pthread_mutex_unlock(&this->joy_mutex_);
}

void HyperionLineTrackerAlgNode::ir_error_callback(const hyperion_infrared::opticalinf::ConstPtr& msg)
{
  ROS_DEBUG("HyperionLineTrackerAlgNode::ir_error_callback: New Message Received");

  this->alg_.lock();
  this->error = msg->sensorinf;
  this->alg_.unlock();
}

void HyperionLineTrackerAlgNode::ir_error_mutex_enter(void)
{
  pthread_mutex_lock(&this->ir_error_mutex_);
}

void HyperionLineTrackerAlgNode::ir_error_mutex_exit(void)
{
  pthread_mutex_unlock(&this->ir_error_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HyperionLineTrackerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void HyperionLineTrackerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HyperionLineTrackerAlgNode>(argc, argv, "hyperion_line_tracker_alg_node");
}
