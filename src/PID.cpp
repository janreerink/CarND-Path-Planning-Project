#include "PID.h"

using namespace std;

/*
* Imported from Udacity self-driving car nanodegree, term 2
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; // differential term, previous error still stored in p-term
  p_error = cte; // proportioanl term
  i_error += cte; // integral term
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}
