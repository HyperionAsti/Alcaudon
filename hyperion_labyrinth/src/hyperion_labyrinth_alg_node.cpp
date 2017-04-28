#include "hyperion_labyrinth_alg_node.h"

HyperionLabyrinthAlgNode::HyperionLabyrinthAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HyperionLabyrinthAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 100;//in [Hz]

  // [init publishers]
  this->stop_publisher_ = this->public_node_handle_.advertise<std_msgs::Bool>("stop", 1);
  this->speeds_publisher_ = this->public_node_handle_.advertise<controlador_motores::Speeds>("speeds", 1);

  // [init subscribers]
  this->joy_subscriber_ = this->public_node_handle_.subscribe("joy", 1, &HyperionLabyrinthAlgNode::joy_callback, this);
  pthread_mutex_init(&this->joy_mutex_,NULL);

  this->distances_subscriber_ = this->public_node_handle_.subscribe("distances", 1, &HyperionLabyrinthAlgNode::distances_callback, this);
  pthread_mutex_init(&this->distances_mutex_,NULL);
  this->error = 0;
  this->stop = true;
  this->front_wall = false;

  // [init services]
  
  // [init clients]
  turn_client_ = this->public_node_handle_.serviceClient<controlador_motores::Turn>("turn");

  
  // [init action servers]
  
  // [init action clients]
}

HyperionLabyrinthAlgNode::~HyperionLabyrinthAlgNode(void)
{
  // [free dynamic memory]
  pthread_mutex_destroy(&this->joy_mutex_);
  pthread_mutex_destroy(&this->distances_mutex_);
  this->stop = true;
  this->front_wall = false;
  this->error = 0;
}

void HyperionLabyrinthAlgNode::mainNodeThread(void)
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
    if (this->front_wall)
    {
      if (this->config_.error_calculation == 1)//left-sided control
        this->turn_srv_.request.right = "right";
      else
        this->turn_srv_.request.right = "left";
      //ROS_INFO("HyperionLabyrinthAlgNode:: Sending New Request!");
      this->alg_.unlock();
      if (turn_client_.call(this->turn_srv_))
        ROS_DEBUG("HyperionLabyrinthAlgNode:: Response: OK");
      else
        ROS_ERROR("HyperionLabyrinthAlgNode:: Failed to Call Server on topic turn ");
      this->alg_.lock();
    }
    else
    {
      this->speeds_msg_.right_speed = (pid_out > 0 ? (this->config_.max_speed - pid_out < this->config_.turn_speed_saturation ? this->config_.turn_speed_saturation : (int)(this->config_.max_speed - pid_out)) : this->config_.max_speed);
      this->speeds_msg_.left_speed = (pid_out > 0 ? this->config_.max_speed : (this->config_.max_speed + pid_out < this->config_.turn_speed_saturation ? this->config_.turn_speed_saturation : (int) (this->config_.max_speed + pid_out)));
    }
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
  
  // [fill srv structure and make request to the server]
  //turn_srv_.request.data = my_var;
  //ROS_INFO("HyperionLabyrinthAlgNode:: Sending New Request!");
  //if (turn_client_.call(turn_srv_))
  //{
    //ROS_INFO("HyperionLabyrinthAlgNode:: Response: %s", turn_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("HyperionLabyrinthAlgNode:: Failed to Call Server on topic turn ");
  //}


  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void HyperionLabyrinthAlgNode::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //ROS_INFO("HyperionLabyrinthAlgNode::joy_callback: New Message Received");
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
  //ROS_INFO("HyperionLabyrinthAlgNode::joy_callback: out");
  joy_mutex_exit();
}

void HyperionLabyrinthAlgNode::joy_mutex_enter(void)
{
  pthread_mutex_lock(&this->joy_mutex_);
}

void HyperionLabyrinthAlgNode::joy_mutex_exit(void)
{
  pthread_mutex_unlock(&this->joy_mutex_);
}

void HyperionLabyrinthAlgNode::distances_callback(const ultrasonido::Distances::ConstPtr& msg)
{
  distances_mutex_enter();
  //ROS_INFO("HyperionLabyrinthAlgNode::distances_callback: New Message Received");
  this->alg_.lock();
  this->front_wall = (msg->distance_front < this->config_.front_wall_dist);
  //ROS_INFO_STREAM("front" << this->front_wall);
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
  //ROS_INFO("HyperionLabyrinthAlgNode::distances_callback: out");
  distances_mutex_exit();
}

void HyperionLabyrinthAlgNode::distances_mutex_enter(void)
{
  pthread_mutex_lock(&this->distances_mutex_);
}

void HyperionLabyrinthAlgNode::distances_mutex_exit(void)
{
  pthread_mutex_unlock(&this->distances_mutex_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HyperionLabyrinthAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void HyperionLabyrinthAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HyperionLabyrinthAlgNode>(argc, argv, "hyperion_labyrinth_alg_node");
}
