#include "rnn_follower.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"

#define STATE_ESTIMATOR 1

// Initializer
rnn_follower::rnn_follower() : Controller() {};

void rnn_follower::bindNorm(float max_command)
{
  float normcom = sqrt(recurrentNN.m_outputNeurons[1] * recurrentNN.m_outputNeurons[1] + recurrentNN.m_outputNeurons[0] * recurrentNN.m_outputNeurons[0]);
  if (normcom > max_command) {
    rnnhandle.commands_lim[0] = recurrentNN.m_outputNeurons[0] * max_command / normcom;
    rnnhandle.commands_lim[1] = recurrentNN.m_outputNeurons[1] * max_command / normcom;
  } else {
    rnnhandle.commands_lim[0] = recurrentNN.m_outputNeurons[0];
    rnnhandle.commands_lim[1] = recurrentNN.m_outputNeurons[1];
  }
}

void rnn_follower::get_velocity_command(const uint16_t ID, float &vx_des, float &vy_des)
{
  // px, py, vx0, vy0 are the inputs to our controller;
  if (ID > 0) {
    uint8_t ID_tracked = ID - 1;

#if STATE_ESTIMATOR
    filter.run(ID, ID_tracked);
    float px = filter.ekf_rl.X[0]; // Relative px
    float py = filter.ekf_rl.X[1]; // Relative py
    float vx0, vy0;
    rotate_xy(filter.ekf_rl.X[6], filter.ekf_rl.X[7], filter.ekf_rl.X[8], vx0, vy0);
#else
    polar2cart(o.request_distance(ID, ID_tracked), o.request_bearing(ID, ID_tracked), px, py);
    rotate_xy(s[ID_tracked]->get_state(2), s[ID_tracked]->get_state(3), -s[ID]->get_state(6), vx0, vy0);
#endif

    std::vector <float> input_vec;
    input_vec.push_back(px);
    input_vec.push_back(py);
    input_vec.push_back(vx0);
    input_vec.push_back(vy0);

    recurrentNN.Evaluate( input_vec );

    bindNorm(0.5);
    vx_des = rnnhandle.commands_lim[0];
    vy_des = rnnhandle.commands_lim[1];
  } else { // random trajectory for the leader
    if ( recurrentNN.m_previous_v_des[0] == 0 && recurrentNN.m_previous_v_des[1] == 0) {
      float v_des_init = rg.uniform_float( 0.1, 0.5 );
      float delta_dir_init = rg.uniform_float( -3.14, 3.14 );
      rotate_xy(v_des_init, 0.0, delta_dir_init, vx_des, vy_des);
    } else {
      if ( rg.uniform_int( 1, 1500 ) > 10 ) {
        vx_des = recurrentNN.m_previous_v_des[0];
        vy_des = recurrentNN.m_previous_v_des[1];
      } else {
        float v_des = rg.uniform_float( 0.1, 0.5 );
        float delta_dir = rg.uniform_float( -3.14 / 4, 3.14 / 4 );
        float prev_v, prev_dir;
        cart2polar( recurrentNN.m_previous_v_des[0], recurrentNN.m_previous_v_des[1], prev_v, prev_dir );
        rotate_xy(v_des, 0.0, prev_dir + delta_dir, vx_des, vy_des);
      }
    }
    recurrentNN.save_velocity_for_random_trajectory( vx_des, vy_des);
  }
} 
