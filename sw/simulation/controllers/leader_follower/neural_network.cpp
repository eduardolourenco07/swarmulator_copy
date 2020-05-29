#include "neural_network.h"
#include <cstring>
#include <fstream>
#include "auxiliary.h"
#include <iostream>
#include "main.h"

Network::Network()
  : m_numInputs( param->num_inputs() )
  , m_numHidden( param->num_hidden() )
  , m_numOutputs( param->num_outputs() )
  , m_recurrentNeurons( m_numHidden * m_numHidden, 0.0 )
{
  InitializeNetwork();
  LoadWeights();
}

void Network::InitializeNetwork()
{
  int32_t const totalNumInputs = m_numInputs + 1;
  int32_t const totalNumHiddens = m_numHidden + 1;

  m_inputNeurons.resize( totalNumInputs );
  m_recurrentNeurons.resize( m_numHidden );
  m_hiddenNeurons.resize( totalNumHiddens);
  m_outputNeurons.resize( m_numOutputs );

  memset( m_inputNeurons.data(), 0, m_inputNeurons.size() * sizeof( float ) );
  memset( m_recurrentNeurons.data(), 0, m_recurrentNeurons.size() * sizeof( float ) );
  memset( m_hiddenNeurons.data(), 0, m_hiddenNeurons.size() * sizeof( float ) );
  memset( m_outputNeurons.data(), 0, m_outputNeurons.size() * sizeof( float ) );

  // Variable for leader
  m_previous_v_des.resize( 2 );
  memset( m_previous_v_des.data(), 0, m_previous_v_des.size() * sizeof( float ) );

  // Set bias values
  m_inputNeurons.back() = 1.0;
  m_hiddenNeurons.back() = 1.0;

  int32_t const numInputHiddenWeights = totalNumInputs * m_numHidden;
  int32_t const numRecurrentWeights = m_numHidden * m_numHidden;
  int32_t const numHiddenOutputWeights = totalNumHiddens * m_numOutputs;
  m_weightsInputHidden.resize( numInputHiddenWeights );
  m_weightsRecurrent.resize( numRecurrentWeights );
  m_weightsHiddenOutput.resize( numHiddenOutputWeights );
}

void Network::LoadWeights( )
{
  int32_t const totalNumInputs = m_numInputs + 1;
  int32_t const totalNumHiddens = m_numHidden + 1;

  int32_t const numInputHiddenWeights = totalNumInputs * m_numHidden;
  int32_t const numRecurrentWeights = m_numHidden * m_numHidden;
  int32_t const numHiddenOutputWeights = totalNumHiddens * m_numOutputs;

  // Load weights
  vector <float> weights;
  if (!strcmp(param->nn_weights().c_str(), "")) {
    weights = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0};
  } else {
    weights = read_array(param->nn_weights());
  }

  int32_t weightIdx = 0;
  for ( int32_t InputHiddenIdx = 0; InputHiddenIdx < numInputHiddenWeights; InputHiddenIdx++ )
  {
    m_weightsInputHidden[InputHiddenIdx] = weights[weightIdx];
    weightIdx++;
  }

  for ( int32_t RecurrentIdx = 0; RecurrentIdx < numRecurrentWeights; RecurrentIdx++)
  {
    m_weightsRecurrent[RecurrentIdx] = weights[weightIdx];
    weightIdx++;
  }

  for ( int32_t HiddenOutputIdx = 0; HiddenOutputIdx < numHiddenOutputWeights; HiddenOutputIdx++)
  {
    m_weightsHiddenOutput[HiddenOutputIdx] = weights[weightIdx];
    weightIdx++;
  }
}

void Network::save_velocity_for_random_trajectory(const float &vx_past, const float &vy_past)
{
  m_previous_v_des[0] = vx_past;
  m_previous_v_des[1] = vy_past;
}

void Network::Evaluate( vector<float> const& input )
{
  // Set input neurons values & recurrent neurons values
  memcpy( m_inputNeurons.data(), input.data(), input.size() * sizeof( float ) );
  memcpy( m_recurrentNeurons.data(), m_hiddenNeurons.data(), ( m_hiddenNeurons.size() - 1 ) * sizeof( float ) );

  // Update Hidden Neurons
  for (int32_t hiddenIdx = 0; hiddenIdx < m_numHidden; hiddenIdx++ )
  {
    m_hiddenNeurons[hiddenIdx] = 0;

    for ( int32_t inputIdx = 0; inputIdx <= m_numInputs; inputIdx++ )
    {
      int32_t const weightIdx = GetInputHiddenWeightIndex( inputIdx, hiddenIdx );
      m_hiddenNeurons[hiddenIdx] += m_inputNeurons[inputIdx] * m_weightsInputHidden[weightIdx];
    }
 
    for (int32_t recurrentIdx = 0; recurrentIdx < m_numHidden; recurrentIdx++ )
    {
      int32_t const weightIdx = GetRecurrentWeightIndex( recurrentIdx, hiddenIdx );
      m_hiddenNeurons[hiddenIdx] += m_recurrentNeurons[recurrentIdx] * m_weightsRecurrent[weightIdx];
    }
    
    // Apply Activation Function
    m_hiddenNeurons[hiddenIdx] = TanHActivationFunction( m_hiddenNeurons[hiddenIdx] );
  }

  // Calculate Output Values 
  for ( int32_t outputIdx = 0; outputIdx < m_numOutputs; outputIdx++ ) 
  {
    m_outputNeurons[outputIdx] = 0;

    for ( int32_t hiddenIdx = 0; hiddenIdx <= m_numHidden; hiddenIdx++ )
    {
      int32_t const weightIdx = GetHiddenOutputWeightIndex( hiddenIdx, outputIdx );
      m_outputNeurons[outputIdx] += m_hiddenNeurons[hiddenIdx] * m_weightsHiddenOutput[weightIdx];
    }

    // Apply Activation Function
    m_outputNeurons[outputIdx] = TanHActivationFunction( m_outputNeurons[outputIdx] );
  }
}

