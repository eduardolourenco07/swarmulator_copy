#include "controller_pattern.h"
#include <algorithm> // std::find
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "trigonometry.h"
#include "auxiliary.h"

controller_pattern::controller_pattern() : controller_lattice_basic()
{
  // Define here the state-action matrix used by the agents
  string s = "./conf/state_action_matrices/state_action_matrix_lineNE.txt";
  t.set_state_action_matrix(s);
  moving_timer = 0;
  beta_des.push_back(0.0);
  beta_des.push_back(atan(_ddes_y / _ddes_x));
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(M_PI / 2.0 + atan(_ddes_x / _ddes_y));
}

void controller_pattern::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0.0;
  v_y = 0.0;

  float timelim = 1.3 * param->simulation_updatefreq();
  float tadj    = timelim * 2.0; // This is actually time for action + t_adj, thus t_adj = time for action
  float twait   = tadj    * 2.0; // This is actually time for action + t_adj + t_wait

  // Initialize local moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rand() % (int)twait;
  }

  vector<bool> state(8, 0);
  vector<int>  state_ID;

  // The ID is just used for simulation purposes
  t.assess_situation(ID, state, state_ID);
  int state_index = bool2int(state);
  // int rot = 0;
  // if (state_index > 0) {
  //   while (!state[0]) {
  //     std::rotate(state.begin(), state.begin() + 1, state.end());
  //     rot++;
  //   }
  // }
  state_index = bool2int(state);

  // Get vector of all neighbors from closest to furthest
  // cout << (int)ID << ": " << state_index << ". ";
  // for (int i = 0; i < state.size(); i++)
  //   std::cout << state[i] << ' ';
  // cout << endl;
  vector<uint> closest = o.request_closest(ID);

  // Can I move or are my neighbors moving?
  bool canImove = check_motion(state_ID);
  if (!canImove) {
    selected_action = -2; // Reset actions
    moving_timer = tadj;  // Reset moving timer
  }

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t.state_action_matrix.find(state_index);
  if (!o.see_if_moving(ID) && state_action_row != t.state_action_matrix.end()) {
    selected_action = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
    // selected_action = wraptosequence(selected_action + 1 + rot, 1, 8) - 1; // North correction
  } else if (!o.see_if_moving(ID)) {
    selected_action = -2;
  }

  // Controller
  moving = false;
  if (canImove) {
    if (selected_action > -1 && moving_timer < timelim) {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    } else {
      get_lattice_motion_all(ID, state_ID, closest, v_x, v_y);
    }
    increase_counter(moving_timer, twait);
  }

}
