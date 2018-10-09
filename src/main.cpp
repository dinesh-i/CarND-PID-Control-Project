#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
//  pid.Init(0.4, 0.001, 3.0);
//  pid.Init(0.3, 0.001, 3.0);
//  pid.Init(0.35, 0.0001, 3.0);

//  double p[] = {0.35, 0.0001, 3.0};
//  pid.Init(p);

//  double p[] = {0.30, 0.0001, 3.0};

//  double p[] = {0.7, 0.0002, 3.5};
//  double p[] = {0.65, 0.0002, 3.5};
//  double p[] = {0.60, 0.0002, 3.5};
//  double p[] = {0.50, 0.0002, 4.0}; // Crossed the lane few times on curves
//  double p[] = {0.40, 0.0002, 4.0}; // Crossed the lane at least 6 times before the bridge
//  double p[] = {0.40, 0.0002, 3.5}; // Crossed the curve so much and got stuck
//  double p[] = {0.40, 0.0002, 3.0};// Crossed the curve so much and got stuck

//  double p[] = {0.40, 0.0002, 5.0};// Crossed the curve so much and got stuck

//  double p[] = {0.7, 0.0002, 3.5}; // Better result. Vehicle is stuck sometimes due to extreme steering value change. Need to fine tune push forward.
  	  	  	  	  	  	  	  	  // Almost completed the track and got stuck before the lap end
//  double p[] = {0.7, 0.0002, 3.25}; // Crossed the curve so much and stagnant move after the bridge
//  double p[] = {0.7, 0.0002, 3.75}; // Better result. COmpleted the lap with few cross on the lane ends.
//  double p[] = {0.7, 0.0002, 4.0}; // Stagnant move before the end of the lap
//  double p[] = {0.7, 0.0003, 3.75}; // Best Result so far. Crossed 1 lap properly and went till the bridge.
//  double p[] = {0.7, 0.0004, 3.75}; // Better result. Almost completes the lap. But moves very slowly when extreme steering values are encountered

  double p[] = {0.7, 0.0003, 3.75};

//  double p[] = {0.20, 0.0001, 3.0};
  double dp[] = {0.5, 0.0001, 0.5};

  pid.Init(p, dp);

  int continuous_steps_with_no_throttle = 0, threshold_of_continuous_steps_with_no_throttle = 30;
  int push_forward = 0;
  double push_forward_throttle_value = 0.5;
  int step = 0;
  int extreme_steer_values_count_in_range = 0;
  int extreme_steer_values_count_in_range_threshold = 25;
  int extreme_steer_values_interval = 50;

  //  Twiddle parameters
	const int max_trial_count = 3;
	const double min_thresold_sum_of_dp = 0.5;
	const int initial_steps_to_skip = 500;
	const int max_steps_to_compute_total_error = 4000;

	int twiddle_current_step = 0;
	int current_trial_count = 0;

	int param_index_to_tune = 0;

	double total_error = 0.0, best_error = 0.0;
	bool first_run = true;
	bool twiddle_is_enabled = false;
	bool added_dp = false, subtracted_dp = false;

//	double best_p[3] = {p[0], p[1], p[2]};
//	best_p[0] = p[0];
//	best_p[1] = p[1];
//	best_p[2] = p[2];

  h.onMessage([&pid, &continuous_steps_with_no_throttle, &threshold_of_continuous_steps_with_no_throttle, &step, &extreme_steer_values_count_in_range, &extreme_steer_values_interval, &extreme_steer_values_count_in_range_threshold, &push_forward, &push_forward_throttle_value, &max_trial_count, &min_thresold_sum_of_dp, &initial_steps_to_skip, &max_steps_to_compute_total_error, &twiddle_current_step, &current_trial_count, &param_index_to_tune, &total_error, &best_error, &first_run, &twiddle_is_enabled, &added_dp, &subtracted_dp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;


//          Twiddle
      	if( twiddle_is_enabled ){

//			  std::cout << "Inside twiddle_is_enabled# " << twiddle_is_enabled << "current_trial_count" << current_trial_count << std::endl;
      		if( current_trial_count > max_trial_count || (pid.GetDpAtIndex(0)+pid.GetDpAtIndex(1)+pid.GetDpAtIndex(2)) < min_thresold_sum_of_dp){
//      			print the result and disable twiddle
//      			std::cout << "Best p values : [" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "]" << std::endl;
      			pid.PrintBestP();
      			twiddle_is_enabled = false;
      		}

      		if( twiddle_current_step > initial_steps_to_skip)
      			total_error += cte;

      		if( twiddle_current_step == (initial_steps_to_skip + max_steps_to_compute_total_error)){
//      			std::cout << "Trial# " << current_trial_count << ", Best p values : [" << best_p[0] << ", " << best_p[1] << ", " << best_p[2] << "]" << std::endl;
      			std::cout << "Trial# " << current_trial_count << std::endl;

      			if( first_run ){
					std::cout << "First Run " << std::endl;
      				first_run = false;
      				best_error = total_error;

      				total_error = 0.0;
      				twiddle_current_step = 0;
      				current_trial_count++;

      				pid.SetBestP(pid.GetPAtIndex(0), pid.GetPAtIndex(1), pid.GetPAtIndex(2));
      				pid.PrintBestP();

      			}
      			else {
					std::cout << "Not First Run" << std::endl;
      				if(added_dp && !subtracted_dp){
						std::cout << "Added But Not Subtracted" << std::endl;

      					if(total_error < best_error){
							best_error = total_error;
							/*best_p[0] = p[0];
							best_p[1] = p[1];
							best_p[2] = p[2];*/
							pid.SetBestP(pid.GetPAtIndex(0), pid.GetPAtIndex(1), pid.GetPAtIndex(2));
							pid.PrintBestP();
//							std::cout << "Best p values : [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl;
//							dp[param_index_to_tune] *= 1.1
							pid.SetDpAtIndex(param_index_to_tune, 1.1);


							param_index_to_tune = (param_index_to_tune + 1)%3;
							twiddle_current_step = 0;
							added_dp = false;
							subtracted_dp = false;
      					}
      					else {
//      						p[param_index_to_tune] -= 2 * dp[param_index_to_tune];
      						pid.SetPAtIndex(param_index_to_tune, (pid.GetPAtIndex(param_index_to_tune) - 2*pid.GetDpAtIndex(param_index_to_tune)));
      						std::cout << "Starting a New Robot " << std::endl;
      						//  Make Robot
//							pid.Init(p, dp);
      						pid.ResetErrors();
							added_dp = subtracted_dp = true;
							total_error = 0.0;
							twiddle_current_step = 0;
							current_trial_count++;
							pid.Restart(ws);
      					}
      				}
      				else if(added_dp && subtracted_dp) {
						std::cout << "Added And Subtracted" << std::endl;
      					if(total_error < best_error){
      						best_error = total_error;
							/*best_p[0] = p[0];
							best_p[1] = p[1];
							best_p[2] = p[2];*/
      						pid.SetBestP(pid.GetPAtIndex(0), pid.GetPAtIndex(1), pid.GetPAtIndex(2));
//							std::cout << "Best p values : [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl;
      						pid.PrintBestP();
//							dp[param_index_to_tune] *= 1.1;
							pid.SetDpAtIndex(param_index_to_tune, 1.1);

							param_index_to_tune = (param_index_to_tune + 1)%3;
							twiddle_current_step = 0;
							added_dp = false;
							subtracted_dp = false;
      					}
      					else {
//      						p[param_index_to_tune] += dp[param_index_to_tune];
//							dp[param_index_to_tune] *= 0.9;
      						pid.SetPAtIndex(param_index_to_tune, (pid.GetDpAtIndex(param_index_to_tune) + pid.GetPAtIndex(param_index_to_tune)));
      						pid.SetDpAtIndex(param_index_to_tune, 0.9);

							param_index_to_tune = (param_index_to_tune + 1)%3;
							twiddle_current_step = 0;
							added_dp = false;
							subtracted_dp = false;
      					}
      				}

      			}
      		}

      		if( twiddle_current_step == 0 && !first_run && !added_dp && !subtracted_dp) {
//      			p[param_index_to_tune] += dp[param_index_to_tune];
      			pid.SetPAtIndex(param_index_to_tune, (pid.GetDpAtIndex(param_index_to_tune) + pid.GetPAtIndex(param_index_to_tune)));
      			added_dp = true;
				std::cout << "Starting a New Robot " << std::endl;
				pid.PrintP();
      			//  Make Robot
//      			pid.Init(p, dp);
				pid.ResetErrors();
      			subtracted_dp = false;
      			total_error = 0.0;
      			twiddle_current_step = 0;
				current_trial_count++;
				pid.Restart(ws);
      		}


      		twiddle_current_step++;

      		if( twiddle_current_step % 500 == 0)
      			std::cout << "Current Step# " << twiddle_current_step << std::endl;

      	}






          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte, ws);
          steer_value = pid.TotalError();

          double steer_value_min_value = -0.9, steer_value_max_value = 0.9;

          json msgJson;
          msgJson["steering_angle"] = steer_value;

//          Push forward is used in situation where the robot is not moving anywhere and is kind of stuck
          if( push_forward > 0 ){
        	  throttle_value = push_forward_throttle_value;
        	  if( push_forward == 1 ){
        		  std::cout << "Pushed forward" << std::endl;
        		  std::cout << "--------------" << std::endl;
        	  }
//        	  else
//        		  std::cout << "Pushing forward" << std::endl;
        	  push_forward--;

        	  /*
//        	  if steering value is outside the threshold update it to be close to the threshold limit
        	  if( steer_value > steer_value_max_value )
        		  steer_value = steer_value_max_value - 0.1;
        	  else if( steer_value < steer_value_min_value )
        		  steer_value = steer_value_min_value + 0.1;
				*/

        	  steer_value = 0.0;

          }
          else if( steer_value >= steer_value_min_value && steer_value <= steer_value_max_value ){
        	  throttle_value = 0.3;
        	  continuous_steps_with_no_throttle = 0;
          }
          else if( ( steer_value > steer_value_max_value || steer_value < steer_value_min_value) ) {
        	  if( continuous_steps_with_no_throttle <= threshold_of_continuous_steps_with_no_throttle){
				  throttle_value = 0.0;
				  continuous_steps_with_no_throttle++;
        	  }
        	  else {
        		  std::cout << "Set to Push forward[No Throttle for threshold no. of times]" << std::endl;
				  throttle_value = 0.4;
				  continuous_steps_with_no_throttle = 0;
				  push_forward = 5;
				  push_forward_throttle_value = 0.5;
        	  }
          }

//          Push forward if extreme steering values occur more than the allowed no. of times making the robot to stand still
          if( step % extreme_steer_values_interval == 0){
        	  extreme_steer_values_count_in_range = 0;
          }

          if( steer_value == -1.0 || steer_value == 1.0)
        	  extreme_steer_values_count_in_range++;

          if( extreme_steer_values_count_in_range >= extreme_steer_values_count_in_range_threshold){
    		  std::cout << "Set to Push forward[Extreme Steer Values for threshold no. of times]" << std::endl;
			  throttle_value = 0.3;
			  continuous_steps_with_no_throttle = 0;
			  push_forward = 5;
			  push_forward_throttle_value = 0.5;

			  extreme_steer_values_count_in_range = 0;
    	  }

          msgJson["throttle"] = throttle_value;
          // DEBUG
//        std::cout << "CTE : " << cte << ", Steering : " << steer_value << ", Throttle : " << throttle_value << std::endl;


          step++;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
