#include "first_order.h"
#include "trigonometry.h"
#include "randomgenerator.h"
#include "draw.h"
#include "main.h"

first_order::first_order(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  random_generator rg;
  orientation = state[6];
  controller->set_saturation(0.5);
  manual = false;
  tau = param->tau() * (0.95 + 0.1*rg.uniform_float(0,1));
}

vector<float> first_order::state_update(vector<float> state)
{
  float vx_des, vy_des = 0;
  float vx_global, vy_global, dpsirate;
  if (!manual) {
    controller->get_velocity_command(ID, vx_des, vy_des); // Command comes out in the local frame
    dpsirate = 0;
  } else {
    vx_des = manualx;
    vy_des = manualy;
    dpsirate = manualpsi_delta;
  }
  controller->saturate(vx_des);
  controller->saturate(vy_des);
#if COMMAND_LOCAL
  rotate_xy(vx_des, vy_des, state[6], vx_global, vy_global);
#else
  vx_global = vx_des;
  vy_global = vy_des;
#endif

  state.at(7) = dpsirate;
  state.at(6) += dpsirate * dt;
  state.at(6) = wrapToPi_f(state[6]); // Orientation
  orientation = state.at(6);

  // Runge-Kutta Integration Variables
  float k_1_vx = -1 * state[2] / (tau + dt) + vx_global / (tau + dt);
  float k_2_vx = -1 * (state[2] + dt * k_1_vx / 2) / (tau + dt) + vx_global / (tau + dt);
  float k_3_vx = -1 * (state[2] + dt * k_2_vx / 2) / (tau + dt) + vx_global / (tau + dt);
  float k_4_vx = -1 * (state[2] + dt * k_3_vx) / (tau + dt) + vx_global / (tau + dt);
  
  float k_1_vy = -1 * state[3] / (tau + dt) + vy_global / (tau + dt);
  float k_2_vy = -1 * (state[3] + dt * k_1_vy / 2) / (tau + dt) + vy_global / (tau + dt);
  float k_3_vy = -1 * (state[3] + dt * k_2_vy / 2) / (tau + dt) + vy_global / (tau + dt);
  float k_4_vy = -1 * (state[3] + dt * k_3_vy) / (tau + dt) + vy_global / (tau + dt);

  float k_1_x = state[2];
  float k_2_x = state[2] + dt * k_1_x / 2;
  float k_3_x = state[2] + dt * k_2_x / 2;
  float k_4_x = state[2] + dt * k_3_x;

  float k_1_y = state[3];
  float k_2_y = state[3] + dt * k_1_y / 2;
  float k_3_y = state[3] + dt * k_2_y / 2;
  float k_4_y = state[3] + dt * k_3_y;

  // Acceleration control
  state.at(4) = (k_1_vx  + 2 * k_2_vx + 2 * k_3_vx + k_4_vx) / 6; // Acceleration global frame
  state.at(5) = (k_1_vy  + 2 * k_2_vy + 2 * k_3_vy + k_4_vy) / 6; // Acceleration global frame
  moving = controller->moving;
  happy = controller->happy;

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x global frame
  state.at(3) += state[5] * dt; // Velocity y global frame

  // Position
  state.at(0) += dt * (k_1_x  + 2 * k_2_x + 2 * k_3_x + k_4_x) / 6; // Position x global frame
  state.at(1) += dt * (k_1_y  + 2 * k_2_y + 2 * k_3_y + k_4_y) / 6; // Position y global frame

  return state;
};

void first_order::animation()
{
  draw d;

  d.triangle(param->scale());
  d.circle_loop(rangesensor);
}
