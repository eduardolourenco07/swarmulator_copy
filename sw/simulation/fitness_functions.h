#ifndef FITNESS_FUNCTIONS_H
#define FITNESS_FUNCTIONS_H

#include "omniscient_observer.h"

/**
 * Mean number of neighbors
 *
 * @return float the mean number of neighbors that each robot has
 */
inline static float mean_number_of_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
    f += (float)closest.size() / (float)s.size();
  }
  return f;
}

/**
 * Mean distance to all
 *
 * @return float the mean distance between all robots
 */
inline static float mean_dist_to_all()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    vector<float> r, b;
    o.relative_location(ID, r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)s.size();
  }
  return f;
}

/**
 * Mean distance to neighbors
 *
 * @return float the mean distance between neighboring robots
 */
inline static float mean_dist_to_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    vector<float> r, b;
    o.relative_location_inrange(ID, rangesensor, r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)s.size();
  }
  return f;
}

/**
 * Connectivity
 * Change f to 0.0 if the graph of the swarm is disconnected, else keeps f
 */
inline static void connectivity_check(float &f)
{
  OmniscientObserver o;
  if (!(o.connected_graph_range(rangesensor))) {
    f = 0.0;
  }
}

inline static uint number_of_clusters()
{
  OmniscientObserver o;
  Graph g(s.size());
  for (size_t ID = 0; ID < s.size(); ID++) {
    vector<uint> neighbors = o.request_closest_inrange(ID, rangesensor);
    for (size_t j = 0; j < neighbors.size(); j++) {
      g.addEdge(ID, neighbors[j]);
    }
  }
  uint a = g.connectedComponents();
  return a;
}

/**
 * Following Performance metric
 *
 * @return float the following performance metric of drone
 */
inline static float following_metric(uint8_t ID, uint8_t ID_tracked)
{
  vector<float> relative_position, leader_velocity;
  float relative_position_norm = 0.;
  float leader_velocity_norm = 0.;
  float angle_cos = 0.;
  float angle = 0.;
  float metric_1 = 0.;
  float metric_2 = 0.;
  OmniscientObserver o;

  relative_position.push_back(o.request_distance_dim(ID, ID_tracked, 0));
  relative_position.push_back(o.request_distance_dim(ID, ID_tracked, 1));
  relative_position_norm = sqrt( 
                                 pow(relative_position[0], 2.0)
                                 + pow(relative_position[1], 2.0)
                               );

  leader_velocity.push_back(s[ID_tracked]->get_state(2));
  leader_velocity.push_back(s[ID_tracked]->get_state(3));
  leader_velocity_norm = sqrt( 
                                 pow(leader_velocity[0], 2.0)
                                 + pow(leader_velocity[1], 2.0)
                               );

  angle_cos = relative_position[0] * leader_velocity[0] + relative_position[1] * leader_velocity[1];
  angle_cos = angle_cos / ( relative_position_norm * leader_velocity_norm );

  angle = acos(angle_cos);

  if (angle != angle) {
    angle = 3.1415 / 2;
  }

  if ( relative_position_norm < 1 ) {
    metric_1 = 1 - 1 / ( 1 + std::exp( 50.0 * ( relative_position_norm - 0.7 ) ) );
  } else {
    metric_1 = 1 - 1 / ( 1 + std::exp( -5.0 * ( relative_position_norm - 2.3 ) + 0.0015 ) );
  }

  metric_2 = 1 - 1 / ( 1 + std::exp( -10.0 * ( angle - 3.1415 / 4 ) ) );

  return metric_1 * metric_2;
}

/**
 * Select a fitness function, or use your own if you want.
 * TODO: Move to controllers so that its defined within a particular controller. It would be far more versatile.
 * @return float fitness
 */
inline static float evaluate_fitness()
{
  float f = 1.;
  // Define the fitness function that you would like to use, or write your own
  /*if (!strcmp(param->fitness().c_str(), "mean_number_of_neighbors"))
  { f = mean_number_of_neighbors();}
  else if (!strcmp(param->fitness().c_str(), "mean_dist_to_neighbors"))
  { f = mean_dist_to_neighbors();}
  else if (!strcmp(param->fitness().c_str(), "aggregation_clusters"))
  { f = 1. / ((float)number_of_clusters() / float(nagents));}
  else if (!strcmp(param->fitness().c_str(), "dispersion_clusters"))
  { f = ((float)number_of_clusters() / float(nagents));}
  else if (!strcmp(param->fitness().c_str(), "aggregation_dist_to_all"))
  { f = 1. / mean_dist_to_all();}
  else if (!strcmp(param->fitness().c_str(), "dispersion_dist_to_all"))
  { f = mean_dist_to_all();}
  else if (!strcmp(param->fitness().c_str(), "food"))
  { f = environment.nest;}
  else if (!strcmp(param->fitness().c_str(), "connected"))
  { connectivity_check(f); }*/

  for (uint8_t i = 1; i < s.size(); i++) {
    f = f * following_metric(i, i - 1);
  }
  return f;
}

#endif
