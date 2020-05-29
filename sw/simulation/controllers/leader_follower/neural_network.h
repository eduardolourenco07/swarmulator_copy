#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H
#include <stdint.h>
#include <vector>
#include <cmath>

using namespace std;

class Network 
{
  inline static float TanHActivationFunction( float x ) {
    return ( std::exp( 2.0 * x ) - 1 ) / ( std::exp( 2.0 * x ) + 1 );
  }


public:
  Network();
  ~Network(){};

  void Evaluate( std::vector<float> const& input );

  std::vector<float> const& GetInputHiddenWeights() const { return m_weightsInputHidden; }
  std::vector<float> const& GetRecurrentWeights() const { return m_weightsRecurrent; }
  std::vector<float> const& GetHiddenOutputWeights() const { return m_weightsHiddenOutput; }

  std::vector<float> m_outputNeurons;
  //stores the previous values of desired velocity for the leader
  std::vector<float> m_previous_v_des;
  void save_velocity_for_random_trajectory(const float &vx_past, const float &vy_past);

private:
  void InitializeNetwork();
  void LoadWeights();

  int32_t GetInputHiddenWeightIndex( int32_t inputIdx, int32_t hiddenIdx ) const { return inputIdx * m_numHidden + hiddenIdx; }
  int32_t GetRecurrentWeightIndex( int32_t recurrentIdx, int32_t hiddenIdx ) const { return recurrentIdx * m_numHidden + hiddenIdx; }
  int32_t GetHiddenOutputWeightIndex( int32_t hiddenIdx, int32_t outputIdx ) const { return hiddenIdx * m_numOutputs + outputIdx; }

  int32_t m_numInputs;
  int32_t m_numHidden;
  int32_t m_numOutputs;

  std::vector<float> m_inputNeurons;
  std::vector<float> m_recurrentNeurons;
  std::vector<float> m_hiddenNeurons;

  std::vector<float>  m_weightsInputHidden;
  std::vector<float>  m_weightsRecurrent;
  std::vector<float>  m_weightsHiddenOutput; 
};

#endif /*NEURAL_NETWORK_H*/
