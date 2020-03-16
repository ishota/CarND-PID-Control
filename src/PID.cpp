#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double discount_rate_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  discount_rate = discount_rate_;
  p_error = 0;
  d_error = 0;
  i_error = 0;
  pre_cte = 0;
  predict_totalError = 0;
  prev_totalError = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = cte - pre_cte;
  i_error += cte;
  pre_cte = cte;

  double next_cte = cte + d_error;
  double next_p_error = next_cte;
  double next_d_error = next_cte - cte;
  double next_i_error = i_error + next_cte;
  predict_totalError = -Kp*next_p_error - Kd*next_d_error - Ki*next_i_error;
}

double PID::TotalError() {
  // return -Kp*p_error - Kd*d_error - Ki*i_error;
  double returned_totalError = predict_totalError * discount_rate - Kp*p_error - Kd*d_error - Ki*i_error + prev_totalError*0.2;
  prev_totalError = returned_totalError;
  return returned_totalError;
}