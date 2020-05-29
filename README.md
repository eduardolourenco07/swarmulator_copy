# Swarmulator

<img align="left" src="https://raw.githubusercontent.com/coppolam/swarmulator/master/logo.png">

* Swarmulator is a lightweight C++ simulator for simulating swarms.
* Swarmulator offers a simple platform to prototype swarm behaviors.
* Swarmulator was designed by Mario Coppola (https://github.com/coppolam) and this repository only makes use of it to implement a neural network controller.
* For more information regarding swarmulator, please visit https://github.com/coppolam/swarmulator .

# Added Files

## sw/simulation/controllers/leader_follower/
	
  * neural_network.h
  * neural_network.cpp
  * rnn_follower.h
  * rnn_follower.cpp

## sw/simulation/agents/

  * first_order/first_order.h
  * first_order/first_order.cpp

## conf/state_action_matrices/

  * weights_rnn.txt (dummy data right now, a solution will be evolved and added)

# Changed Files

## conf/

  * parameters.xsd
  * parameters.xml

## sw/simulation

  * fitness_functions.h
  * simulation_thread.h
