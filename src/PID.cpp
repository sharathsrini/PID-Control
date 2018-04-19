#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  propotional_error = 0.0;
  integral_error = 0.0;
  derivative_error = 0.0;
  first = true;

}

void PID::UpdateError(double cte) {

  integral_error += cte;

  if (first){
    derivative_error = 0;
    first = false;
  }
  else
    derivative_error = cte - propotional_error;

  propotional_error = cte;



}

double PID::TotalError() {
  return propotional_error * Kp + integral_error * Ki + derivative_error * Kd;
}
