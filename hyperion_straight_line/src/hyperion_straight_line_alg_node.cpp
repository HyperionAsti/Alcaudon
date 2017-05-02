#include "hyperion_straight_line_alg_node.h"

HyperionStraightLineAlgNode::HyperionStraightLineAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HyperionStraightLineAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = this->config_.pid_freq;//in [Hz]
  this->loop_rate_ = 100;//in [Hz]

  // [init publishers]
  this->stop_publisher_ = this->public_node_handle_.advertise<std_msgs::Bool>("stop", 1);
  this->speeds_publisher_ = this->public_node_handle_.advertise<hyperion_motor_driver::Speeds>("speeds", 1);
  
  // [init subscribers]
  this->joy_subscriber_ = this->public_node_handle_.subscribe("joy", 1, &HyperionStraightLineAlgNode::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);

  this->distances_subscriber_ = this->public_node_handle_.subscribe("distances", 1, &HyperionStraightLineAlgNode::distances_callback, this);
  pthread_mutex_init(&this->distances_mutex_,NULL);
  this->error = 0;
  this->stop = true;
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

HyperionStraightLineAlgNode::~HyperionStraightLineAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->joy_mutex_);
  pthread_mutex_destroy(&this->distances_mutex_);
  this->stop = true;
  this->error = 0;
}

void HyperionStraightLineAlgNode::mainNodeThread(void)
{
  joy_mutex_enter();
  distances_mutex_enter();
  this->alg_.lock(); 
  //ROS_INFO("hola");
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

  //ROS_INFO("adios");
  this->alg_.unlock();
  distances_mutex_exit();
  joy_mutex_exit();
  this->speeds_publisher_.publish(this->speeds_msg_);
  // [fill msg structures]
  // Initialize the topic message structure
  //this->stop_Bool_msg_.data = my_var;

  // Initialize the topic message structure
  //this->speeds_Speeds_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->stop_publisher_.publish(this->stop_Bool_msg_);

  // Uncomment the following line to publish the topic message
  //this->speeds_publisher_.publish(this->speeds_Speeds_msg_);

}

/*  [subscriber callbacks] */
void HyperionStraightLineAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //ROS_INFO("HyperionStraightLineAlgNode::joy_callback: New Message Received");
  joy_mutex_enter();
  this->alg_.lock();
  static bool button_pressed = false;
  if (msg->buttons[this->config_.stop_button] == 1 && button_pressed == false)
  {
    button_pressed = true;
    this->stop = !(this->stop);

    this->stop_Bool_msg_.data = this->stop;

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
    this->stop_publisher_.publish(this->stop_Bool_msg_);  
    //ROS_INFO_STREAM("stop = " << this->stop);
  }
  else if (msg->buttons[this->config_.stop_button] == 0 && button_pressed == true)
  {
    button_pressed = false;
    this->alg_.unlock();
  }
  else
    this->alg_.unlock();
  //ROS_INFO("HyperionStraightLineAlgNode::joy_callback: out");
  joy_mutex_exit();
}

void HyperionStraightLineAlgNode::joy_mutex_enter(void)
{
  pthread_mutex_lock(&this->joy_mutex_);
}

void HyperionStraightLineAlgNode::joy_mutex_exit(void)
{
  pthread_mutex_unlock(&this->joy_mutex_);
}

void HyperionStraightLineAlgNode::distances_callback(const hyperion_ultrasound::Distances::ConstPtr& msg)
{
  distances_mutex_enter();
  //ROS_INFO("HyperionStraightLineAlgNode::distances_callback: New Message Received");
  this->alg_.lock();
  switch (this->config_.error_calculation)
  {
    case 0://two-sided
      this->error = msg->distance_right - msg->distance_left;
    break;
    case 1://left-sided
      this->error = 2*(this->config_.center_dist - msg->distance_left);
    break;
    case 2://right-sided
      this->error = 2*(msg->distance_right - this->config_.center_dist);
    break;
  }
  this->alg_.unlock();
  //ROS_INFO("HyperionStraightLineAlgNode::distances_callback: out");
  distances_mutex_exit();
}

void HyperionStraightLineAlgNode::distances_mutex_enter(void)
{
  pthread_mutex_lock(&this->distances_mutex_);
}

void HyperionStraightLineAlgNode::distances_mutex_exit(void)
{
  pthread_mutex_unlock(&this->distances_mutex_);
}


/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HyperionStraightLineAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void HyperionStraightLineAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HyperionStraightLineAlgNode>(argc, argv, "hyperion_straight_line_alg_node");
}
