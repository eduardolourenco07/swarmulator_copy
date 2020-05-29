#ifndef RNN_FOLLOWER_H
#define RNN_FOLLOWER_H
#include "controller.h"
#include "ekf_state_estimator.h"
#include "neural_network.h"
#include "randomgenerator.h"

using namespace std;

typedef struct rnnhandler {
  float commands_lim[2];
} rnnhandler;

class rnn_follower : public Controller
{
  rnnhandler rnnhandle;
  ekf_state_estimator filter;
  Network recurrentNN;
  random_generator rg;

public:
  rnn_follower();
  ~rnn_follower() {};

  //bool ndi_follow_leader(void);
  void bindNorm(float max_command);
  virtual void get_velocity_command(const uint16_t ID, float &x_des, float &vy_des);
};

#endif /*RNN_FOLLOWER_H*/
