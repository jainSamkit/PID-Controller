#include "PID.h"
#include<numeric>
#include<iostream>
#include<vector>
#include<math.h>
#include <uWS/uWS.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {

	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	/*For PID*/
	// p = {Kp,Kd,Ki};
	
	/*For PD*/
	p = {Kp,Kd}; 

	/*For PID*/
	// dp = {1,1,0.001};
	
	/*For PD*/
	dp = {1.0,1.0};


	num_steps = 15;
	steps = 0;
	num_eval = 25;

	p_error=0;
	i_error = 0;
	d_error =0;
	tried_adding = false;
	tried_subtracting = false;
	tolerance = 0.01;

	is_twiddle = false;
	first_time = true;
	in_process = false;
	resample = 0;
	prev_cte = 0;
	first_iteration = true;
}

void PID::Update_Ks()
{
	Kp = p[0];
	Kd = p[1];
	// Ki = p[2];
} 

void PID::UpdateCoefficients()
{
	
	if(tolerance_error()>tolerance)
	{
		if(!tried_adding && !in_process)
		{
			p[resample] +=dp[resample];
			tried_adding= true;
			in_process = true;
			return;
		}
		if(err<best_err && tried_adding && !tried_subtracting)
		{
			best_err = err;
			dp[resample] *=1.1;
			in_process = false;
			tried_adding = false;
		}
		else
		{
			if(!tried_subtracting)
			{
				p[resample]-=2*dp[resample];
				tried_subtracting = true;
				in_process = true;
				return;
			}
			if(err<best_err)
			{
				best_err = err;
				dp[resample]*=1.1;
				in_process = false;
				tried_subtracting = false;
			}
			else
			{
				p[resample]+=dp[resample];
				dp[resample]*=0.9;
				tried_subtracting = false;
				tried_adding= false;
				in_process = false;
			}
		}
		if(!tried_adding && !tried_subtracting)
		{
			// resample=(resample+1)%3;
			resample=(resample+1)%2;
		}
	}
}

void PID::UpdateError(double cte) {
	p_error = cte;
	d_error = cte - prev_cte;
	// i_error+=cte;
	i_error = 0;
}

double PID::control(double cte_)
{
	double steer;
	UpdateError(cte_);
	if(!is_twiddle)
	{
		if(first_time)
		{
			if(first_iteration)
			{
				d_error = 0; 
				first_iteration = false;
			}
			steps+=1;
			if(steps%(num_eval+num_steps)>num_steps)
			{
				err+=pow(cte_,2);
			}

				// For PID
			steer = -Kd * d_error - Kp * p_error - Ki * i_error;
			
			
			prev_cte = cte_;
			if(steps%(num_steps+num_eval)==0)
			{
				steps=0;
				first_time = false;
				err /= num_eval;
				best_err = err;
				cout<<"Printing the best error"<<" "<<best_err<<endl;
				cout<<endl;
			}
			return steer;
		}
		if(tolerance_error()>tolerance)
		{
			UpdateError(cte_);
			if(!in_process)
			{
				UpdateCoefficients();
				Update_Ks();
			}

			if(steps==0)
			{
				p_error = cte_;
				d_error = 0;
				i_error = cte_;
				err = 0;
			}
			steps+=1;
			if(steps%(num_steps+num_eval) == 0 && in_process)
			{
				err /=num_eval;
				cout<<"Printing the error"<<" "<<err<<endl;
				cout<<"Printing the best error"<<" "<<best_err<<endl;
				// cout<<"Printing the parameters"<<endl;
				
				// // cout<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
				// cout<<p[0]<<" "<<p[1]<<endl;

				UpdateCoefficients();
				Update_Ks();
				steps = 0;

				/*For PID*/
				cout<<"Printing the sum of the tolerances"<<" "<<dp[0]+dp[1]<<endl;

				/*For PD*/
				// cout<<"Printing the sum of the tolerances"<<" "<<dp[0]+dp[1]<<endl;
				cout<<endl;
			}
			prev_cte = cte_;

			/*For PID*/
			steer = -Kd * d_error - Kp * p_error - Ki * i_error;

			/*For PD*/
			// steer = -Kd * d_error - Kp * p_error;

			if(steps%(num_steps+num_eval) > num_steps)
			{
				err+= pow(cte_,2);
			}
		}
		else
		{
			is_twiddle = true;
			cout<<Kd<<" "<<Kp<<endl;
			steer = -Kd * d_error - Kp * p_error -  Ki * i_error; 
			prev_cte = cte_;
		}
	}
	else
	{
		// if(!is_twiddle)
		// {
		// 	cout<<Kd<<" "<<Kp<<endl;
		// 	is_twiddle = true;
		// }

		/*For PID*/
		steer = -Kd * d_error - Kp * p_error - Ki * i_error;
		prev_cte = cte_;

		// /*For PD*/
		// steer = -Kd * d_error - Kp * p_error;
	}
	return steer;
}

double PID::tolerance_error()
{
	/*For PD*/
	return dp[0] + dp[1];

	/*For PID*/
	// return dp[0] + dp[1] + dp[2];
}
