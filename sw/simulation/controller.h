#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <algorithm>
#include <stdint.h>
#include "omniscient_observer.h"

using namespace std;

/**
 * This is a parent class for a controller.
 * All controllers are children functions that must contain an implementation of get_velocity_command, which is declared virtual here.
 */
class Controller
{
protected:
  OmniscientObserver o;
  random_generator rg;

public:
  /**
   * Constructor
   */
  Controller();

  /**
   * Destructor
   */
  ~Controller() {};
  float _ddes_x; // Desired equilibrium distance_x
  float _ddes_y; // Desired equilibrium distance_y
  float _kr; // Repulsion gain
  float _ka; // Attraction gain
  bool  saturation; // Define whether the controls are saturated
  float saturation_limits; // Define the saturation of the controls
  bool  moving; // Internal state of whether the robot is actively moving or not
  bool happy;

  /**
   * Set the speed saturation to true and set the saturation limits.
   * If this is not used then the saturation won't take place.
   *
   * @param saturation_limits The limit of the saturation (velocity)
   */
  void set_saturation(const float &saturation_limits);

  /**
   * Function to saturate the commands
   *
   * @param f Value to be saturated
   */
  void saturate(float &f);

  /**
   * Attraction function
   * This is defined here since it is a basic behavior for collision avoidance
   * which many higher level controllers may want to use, although it does not have to be.
   * @param u Distance to neighbor
   * @return float Attraction speed
   */
  float f_attraction(float u);

  /**
   * Attraction function
   * This is defined here since it is a basic behavior for collision avoidance.
   * which many higher level controllers may want to use, although it does not have to be.
   * @param u Distance to neighbor
   * @return float Attraction speed
   */
  float f_attraction_equilibrium(float u, float eq_distance);

  /**
   * Repulsion function.
   * This is defined here since it is a basic behavior for collision avoidance
   * which many higher level controllers may want to use, although it does not have to be.
   *
   * @param u Distance to other robot
   * @return Repulsion velocity component
   */
  float f_repulsion(float u);

  /**
   * Implementation of lattice behavior.
   *
   * @param ID
   * @param state_ID
   * @param v_x
   * @param v_y
   */
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);

  /**
   * @brief Get the lattice motion all object
   *
   * @param ID
   * @param v_x
   * @param v_y
   */
  void get_lattice_motion_all(const int &ID, float &v_x, float &v_y);

  /**
   * @brief Get the attraction velocity object
   *
   * @param u
   * @return float
   */
  float get_attraction_velocity(float u);

  /**
   * Virtual function to be implemented by child class that defines the controller
   * The outputs are the desired v_x and v_y velocities
   *
   * @param ID The ID of the robot to control
   * @param v_x The desired velocity in v_x (to be set in this function)
   * @param v_y The desired velocity in v_y (to be set in this function)
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y) = 0;

  /**
   * General wall avoidance function to use within get_velocity_command() to instigate a wall avoidance maneuver
   *
   * @param ID The ID of the robot to control
   * @param v_x The desired velocity in v_x (amended in this function)
   * @param v_y The desired velocity in v_y (amended in this function)
   */
  bool wall_avoidance(const uint16_t ID, float &v_x, float &v_y);
  bool wall_avoidance_t(const uint16_t ID, float &v_x, float &v_y);
};


#endif /*CONTROLLER_H*/
