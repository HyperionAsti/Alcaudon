#include "hyperion_tests_alg_node.h"

HyperionTestsAlgNode::HyperionTestsAlgNode(void) :
  algorithm_base::IriBaseAlgorithm<HyperionTestsAlgorithm>()
{
  //init class attributes if necessary
  this->loop_rate_ = 5;//in [Hz]

  // [init publishers]
  this->ir_error_publisher_ = this->public_node_handle_.advertise<infrarrojos::opticalinf>("ir_error", 1);
  this->us_distances_publisher_ = this->public_node_handle_.advertise<ultrasonido::Distances>("us_distances", 1);
  
  // [init subscribers]
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  this->right_dist = 35.0;
  this->left_dist = 35.0;
  this->ir_err = 0.0;
}

HyperionTestsAlgNode::~HyperionTestsAlgNode(void)
{
  // [free dynamic memory]
}

void HyperionTestsAlgNode::mainNodeThread(void)
{
  static uint8_t count = 0;

  if (count < 85)
  {
    this->right_dist -= 0.2;
    this->left_dist += 0.2;
    this->ir_err += 0.1;
    count++;
    this->us_distances_Distances_msg_.distance_left = this->left_dist;
    this->us_distances_Distances_msg_.distance_right = this->right_dist;
    this->us_distances_publisher_.publish(this->us_distances_Distances_msg_);
    this->ir_error_opticalinf_msg_.sensorinf = this->ir_err;
    this->ir_error_publisher_.publish(this->ir_error_opticalinf_msg_);
  }
  else if (count < 255)
  {
    this->right_dist += 0.2;
    this->left_dist -= 0.2;
    this->ir_err -= 0.1;
    count++;
    this->us_distances_Distances_msg_.distance_left = this->left_dist;
    this->us_distances_Distances_msg_.distance_right = this->right_dist;
    this->us_distances_publisher_.publish(this->us_distances_Distances_msg_);
    this->ir_error_opticalinf_msg_.sensorinf = this->ir_err;
    this->ir_error_publisher_.publish(this->ir_error_opticalinf_msg_);
  }
  // [fill msg structures]
  // Initialize the topic message structure
  //this->ir_error_opticalinf_msg_.data = my_var;

  // Initialize the topic message structure
  //this->us_distances_Distances_msg_.data = my_var;

  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  // Uncomment the following line to publish the topic message
  //this->ir_error_publisher_.publish(this->ir_error_opticalinf_msg_);

  // Uncomment the following line to publish the topic message
  //this->us_distances_publisher_.publish(this->us_distances_Distances_msg_);

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HyperionTestsAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_=config;
  this->alg_.unlock();
}

void HyperionTestsAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<HyperionTestsAlgNode>(argc, argv, "hyperion_tests_alg_node");
}
