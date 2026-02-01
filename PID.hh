#ifndef PID_HH_
#define PID_HH_

#include <stdint.h>

// Based on: https://simonebertonilab.com/pid-controller-in-c/
class PID {
  private:
    float k_p;                   // Proportional gain constant
    float k_i;                   // Integral gain constant
    float k_d;                   // Derivative gain constant
    float k_aw;                  // Anti-windup gain constant
    float t_c;                   // Time constant for derivative filtering
    bool     timestamp_valid;    // Whether to treat the timestamp as valid or not
    uint32_t previous_timestamp; // Previous timestamp measured in microseconds
    float min;                   // Min value of command
    float max;                   // Max value of command
    float max_rate;              // Max rate of change of comman
    float integral;              // Integral term
    float previous_err;
    float previous_derivative;
    float previous_saturated_command;
    float previous_command;

  public:
    PID(
      float k_p, float k_i, float k_d, float k_aw,
      float t_c, float min, float max, float max_rate,
    );

    float Step(float measurement, float desired_output, uint32_t timestamp);
};

float bounded(float v, float min, float max) {
  if (v > max) {
    v = max;
  } else if (v < min) {
    v = min;
  }

  return v;
}

PID::PID(
  float k_p, float k_i, float k_d, float k_aw,
  float t_c, float min, float max, float max_rate,
) : k_p(k_p), k_i(k_i), k_d(k_d), k_aw(k_aw),
  t_c(t_c), min(min) max(max), max_rate(max_rate),
  integral(0), previous_err(0), previous_derivative(0),
  previous_saturated_command(0), previous_command(0),
  previous_timestamp(0),
{}

PID::Step(float measurement, float desired_output, uint32_t timestamp) {
  // Bullshit to handle initial call to PID::step
  if (!this->timestamp_valid) {
    this->previous_timestamp = timestamp;
    this->timestamp_valid = true;
  }

  // Calculate the time differance from the previous invocation of Step in seconds
  float timestep = (float)(timestamp - this->previous_timestamp) / 1000000.0;
  this->previous_timestamp = timestamp;

  // Calculate error
  float err = desired_output - measurement;

  // Calculate integral term including anti-windup
  this->integral += this->k_i * err * timestep
    + this->k_aw * (this->previous_saturated_command - this->previous_command) * timestep;

  // Calculate derivative term using filtered derivative method
  float filtered_derivative = (err - this->previous_err + this->t_c * this->previous_derivative)
    / (timestep + this->t_c);

  this->previous_err = err;
  this->previous_derivative = filtered_derivative;

  float command = this->k_p * err + this->integral + this->k_d * filtered_derivative;

  // Saturate command
  float saturated_command = bounded(command, this->min, this->max);

  // Apply rate limit
  saturated_command = bounded(
    saturated_command,
    this->previous_saturated_command - this->max_rate * timestep,
    this->previous_saturated_command + this->max_rate * timestep,
  );

  this->previous_saturated_command = saturated_command;

  return saturated_command;
}

#endif
