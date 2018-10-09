#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double p[3];

  /*
  * Coefficients
  */
  double dp[3];

  double best_p[3];

//  Twiddle parameters
	int max_trial_count;
	int min_thresold_sum_of_dp;
	int initial_steps_to_skip;
	int current_step;
	int max_steps_to_compute_total_error;
	int current_trial_count;

	int param_index_to_tune;

	double total_error, best_error;
	bool first_run;
	bool twiddle_is_enabled;
	bool added_dp, subtracted_dp;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double p[]);

  void Init( double p[], double dp[] );

  void SetBestP(double p, double i, double d);

  void PrintP();

  void PrintDP();

  void PrintBestP();


  void ResetErrors();

  void SetDpAtIndex(int index, double value);

  double GetDpAtIndex(int index);

  void SetPAtIndex(int index, double value);

  double GetPAtIndex(int index);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, uWS::WebSocket<uWS::SERVER> ws);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

//  Restart from the initial position
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
