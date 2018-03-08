#include "PID.h"
#include <math.h>
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError(double limit) {
  double output;
  output = -(Kp * p_error + Ki * i_error + Kd * d_error);
  if (limit > 0){
    output = max(-limit, min(limit, output));
    //anti wind-up
    if (Ki > 0) i_error = - (output + Kp * p_error + Kd * d_error) / Ki;
  }
  return output;
}
