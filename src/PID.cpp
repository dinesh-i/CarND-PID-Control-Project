#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this -> Kp = Kp;
	this -> Ki = Ki;
	this -> Kd = Kd;

	p_error = i_error = d_error = 0.0;
}

void PID::UpdateError(double cte) {
	p_error = cte;
	d_error = cte - d_error;
	i_error += cte;
}

double PID::TotalError() {
	double result = - Kp * p_error - Kd * d_error - Ki * i_error;
	if( result < -1.0 )
		return -1.0;
	if( result > 1.0 )
		return 1.0;
	return result;
}

