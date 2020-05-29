#ifndef SIMULATION_THREAD_H
#define SIMULATION_THREAD_H

#include <numeric>
#include <cctype>
#include <algorithm>
#include <thread>
#include <cstdlib> // system, NULL, EXIT_FAILURE
#include <iostream>
#include <sstream> // std::stringstream, std::stringbuf
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "main.h"
#include "randomgenerator.h"
#include "terminalinfo.h"
#include "agent_thread.h"
#include "drawingparams.h"
#include "settings.h"
#include "environment.h"
#include "fitness_functions.h"
#include "fifo.h"

/**
 * Extract the number of agents from the argument list.
 * Else, return an error.
 *
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void read_argv(int argc, char *argv[])
{
  if (argc <= 1) {
    terminalinfo::error_msg("Please specify the number of agents.");
  } else {
    nagents = stoi(argv[1]);
  }
}

/**
 * This function initiates the simulation.
 * All agents in the beginning initiate randomly with a mean position around the (0,0) point.
 * Once the vector of agents is created, each agent is launched in a separate thread.
 *
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void main_simulation_thread(int argc, char *argv[], string id)
{
  terminalinfo::info_msg("Simulation started.");
  read_argv(argc, argv); // Read the number of agents from the argument input
  random_generator rg;
  fifo f(id); // Open FIFO pipe


  // Generate the random initial positions with (0,0) mean and 0.5 standard deviation
  if (nagents > 0) {
#ifdef SEQUENTIAL
    vector<float> st = environment.start();
    vector<float> x0 = rg.uniform_float_vector(nagents, st[1] - 0.1, st[1] + 0.1);
    vector<float> y0 = rg.uniform_float_vector(nagents, st[0] - 0.1, st[0] + 0.1);
#else
    float spread = environment.limits(); // default // TODO: Spread randomly within an arbitray arena
    vector<float> x0 = rg.uniform_float_vector(nagents, -3, 3);
    vector<float> y0 = rg.uniform_float_vector(nagents, -3, 3);
#endif
    vector<float> t0 = rg.uniform_float_vector(nagents, -M_PI, M_PI);
    // Generate the agent models
#ifdef SEQUENTIAL
    uint ID = 0;
    float t_created = -SEQUENTIAL - 1; // so that first agent is created at time - 9,9
#else
    for (uint16_t ID = 0; ID < nagents; ID++) {
      vector<float> state = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
      create_new_agent(ID, state); // Create agent
    }
#endif
  }

  // vector with #points,sum,previous value, used for computing fitness
  vector<float> fitness;
  fitness.push_back(0);
  fitness.push_back(0);
  fitness.push_back(0);
  float fitness_aux = 0.;
 
  // Keep global clock running.
  // This is only used by the animation and the logger.
  // The robots operate by their own detached thread clock.
  while (program_running) {
    if (!paused) {
#ifdef SEQUENTIAL
      if (simtime_seconds > t_created + SEQUENTIAL && ID < nagents) {
        vector<float> state = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
        create_new_agent(ID, state); // Create agent
        t_created = simtime_seconds;
        ID++;
      }
#endif
      // Real time evaluation of fitness
      if ( fitness[0] == 0 ) {
        fitness_aux = evaluate_fitness();
        fitness[0] += 1;
        fitness[2] = fitness_aux;
      } else {
        fitness_aux = evaluate_fitness();
        fitness[0] += 1;
        fitness[1] += ( fitness[2] + fitness_aux ) / 2.0;
        fitness[2] = fitness_aux;
      }
      // Runtime finish evolution
      if (param->time_limit() > 0.0) {
        if (simtime_seconds > param->time_limit()) { // Quit after a certain amount of time
          mtx.lock(); // Done
          terminalinfo::debug_msg("Sending message");
          f.send(evaluate_fitness());
          cout << "Final Fitness: " << fitness[1] / fitness[0] << "\n";
          mtx.unlock();
          program_running = false;
        }
      }
    }
  };

}
#endif /*SIMULATION_THREAD_H*/
