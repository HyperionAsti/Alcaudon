#include "hyperion_controller_alg.h"

HyperionControllerAlgorithm::HyperionControllerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

HyperionControllerAlgorithm::~HyperionControllerAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void HyperionControllerAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// HyperionControllerAlgorithm Public API
