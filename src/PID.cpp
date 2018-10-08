#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double p[]) {
//	this->Kp = Kp;
//	this->Ki = Ki;
//	this->Kd = Kd;

	this->p[0] = p[0];
	this->p[1] = p[1];
	this->p[2] = p[2];

	p_error = i_error = d_error = 0.0;

	twiddle_is_enabled = false;
}

void PID::Init( double p[], double dp[] ) {

	this->p[0] = p[0];
	this->p[1] = p[1];
	this->p[2] = p[2];

	this->dp[0] = dp[0];
	this->dp[1] = dp[1];
	this->dp[2] = dp[2];

	p_error = i_error = d_error = 0.0;

}

void PID::ResetErrors() {
	p_error = i_error = d_error = 0.0;
}

void PID::UpdateError(double cte, uWS::WebSocket<uWS::SERVER> ws) {
	p_error = cte;
	d_error = cte - d_error;
	i_error += cte;
}

double PID::TotalError() {
	double result = -p[0] * p_error - p[2] * d_error - p[1] * i_error;
	if (result < -1.0)
		return -1.0;
	if (result > 1.0)
		return 1.0;
	return result;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
	std::string reset_msg = "42[\"reset\",{}]";
	ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

