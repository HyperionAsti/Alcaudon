#include "hyperion_controller_alg_node.h"

HyperionControllerAlgNode::HyperionControllerAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HyperionControllerAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 100;//in [Hz]

  // [init publishers]
  this->speeds_publisher_ = this->public_node_handle_.advertise<controlador_motores::Speeds>("speeds", 1);

  // [init subscribers]
  this->controller_subscriber_ = this->public_node_handle_.subscribe("controller", 1, &HyperionControllerAlgNode::controller_callback, this);
  pthread_mutex_init(&this->controller_mutex_,NULL);

  // [init services]
  
  // [init clients]
  gripper_client_ = this->public_node_handle_.serviceClient<shot_ball::Gancho>("gripper");

  
  // [init action servers]
  
  // [init action clients]
}

HyperionControllerAlgNode::~HyperionControllerAlgNode(void)
{
  // [free dynamic memory]
}

void HyperionControllerAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  //gripper_srv_.request.data = my_var;
  //ROS_INFO("HyperionControllerAlgNode:: Sending New Request!");
  //if (gripper_client_.call(gripper_srv_))
  //{
    //ROS_INFO("HyperionControllerAlgNode:: Response: %s", gripper_srv_.response.result);
  //}
  //else
  //{
    //ROS_INFO("HyperionControllerAlgNode:: Failed to Call Server on topic gripper ");
  //}


  
  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void HyperionControllerAlgNode::controller_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //ROS_INFO("HyperionControllerAlgNode::controller_callback: New Message Received");

  //use appropiate mutex to shared variables if necessary
  this->alg_.lock();
  //this->controller_mutex_enter();
  if (fabs(msg->axes[1]) <= this->config_.axes_deadzone)//It has to turn on the site
  {
    if(fabs(msg->axes[2]) <= this->config_.axes_deadzone)//Stop
    {
      this->speeds_msg_.right_speed = 0;
      this->speeds_msg_.left_speed = 0;
    }
    else
      {
        this->speeds_msg_.right_speed = (int) (msg->axes[2]*this->config_.max_speed);
        this->speeds_msg_.left_speed = this->speeds_msg_.right_speed * -1;
      }
  }
  else//It's a curve
  {
    this->speeds_msg_.right_speed = (int) (msg->axes[1]*this->config_.max_speed);
    this->speeds_msg_.left_speed = this->speeds_msg_.right_speed;
    if (fabs(msg->axes[2]) > this->config_.axes_deadzone)
      (msg->axes[2] < 0 ? this->speeds_msg_.right_speed += (int) this->speeds_msg_.right_speed*msg->axes[2] : this->speeds_msg_.left_speed -= (int) this->speeds_msg_.left_speed*msg->axes[2]);
  }

  if (this->config_.bowling)
  {
    if(msg->buttons[1] == 1)//X button
      this->gripper_srv_.request.estado = "R";
    else if (msg->buttons[3] == 1)//triangle button
      this->gripper_srv_.request.estado = "A";
    else if (msg->buttons[5] == 1)//R1 button
      this->gripper_srv_.request.estado = "D";
    //ROS_INFO("HyperionControllerAlgNode:: Sending New Request!");
    this->alg_.unlock();
    if (gripper_client_.call(this->gripper_srv_))
    {
      ROS_INFO("HyperionControllerAlgNode:: Response: ok");
    }
    else
    {
      ROS_ERROR("HyperionControllerAlgNode:: Failed to Call Server on topic gripper ");
    }
  }
  else
    this->alg_.unlock();
  this->speeds_publisher_.publish(this->speeds_msg_);
  //std::cout << msg->data << std::endl;
  //unlock previously blocked shared variables
  //this->alg_.unlock();
  //this->controller_mutex_exit();
}

void HyperionControllerAlgNode::controller_mutex_enter(void)
{
  pthread_mutex_lock(&this->controller_mutex_);
}

void HyperionControllerAlgNode::controller_mutex_exit(void)
{
  pthread_mutex_unlock(&this->controller_mutex_);
}
/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HyperionControllerAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void HyperionControllerAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HyperionControllerAlgNode>(argc, argv, "hyperion_controller_alg_node");
}
