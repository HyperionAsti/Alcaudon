#include "hyperion_tests_alg.h"

HyperionTestsAlgorithm::HyperionTestsAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

HyperionTestsAlgorithm::~HyperionTestsAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void HyperionTestsAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// HyperionTestsAlgorithm Public API
