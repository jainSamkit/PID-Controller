#ifndef PID_H
#define PID_H

#include<vector>
#include <uWS/uWS.h>
#include<iostream>
using namespace std;
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
  double Kp;
  double Ki;
  double Kd;

  /*Vector of changes*/
  vector<double>dp;
  vector<double>p;

  double prev_cte;
  int steps;

  double best_err;

  /*to check the number of steps*/
  int num_steps;

  /* Adding the errors*/
  int num_eval;

  /* to check the tolerance*/
  double tolerance ;

  /*to optimize parameters*/
  bool is_twiddle;

  bool first_iteration;

  /*to set best_err and to run evaluation for the initially set parameters*/
  bool first_time;

  bool tried_adding;

  bool tried_subtracting;

  bool in_process;

  int resample;

  double err;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*Update coefficients*/
  void UpdateCoefficients();

  /*set_cte*/
  // void set_cte(double cte_);

  /*control*/
  double control(double cte_);

  void Update_Ks();

  void restart(uWS::WebSocket<uWS::SERVER> ws);

  double tolerance_error();
};

#endif /* PID_H */
