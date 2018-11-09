#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	double temp_dp = 0.1;

	// check if the gains are passed with values so the twiddle algo. should not execute
	if((Kp || Ki || Kd) != 0)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;

		twiddle = false;
	}
	else
	{
		cout<<"twiddle tuning"<<endl;

		this->Kp = 0;
		this->Ki = 0;
		this->Kd = 0;

		// set dp for Kp with 1.0
		dp.push_back(temp_dp);

		temp_dp = 0;
		dp.push_back(temp_dp);
		temp_dp = 0;
		dp.push_back(temp_dp);

		//init p vector
		temp_dp = 0;
		p.push_back(temp_dp);
		p.push_back(temp_dp);
		p.push_back(temp_dp);


		//set gain state for PID gains
		gain_state tmp = STEP_UP;

		tuning_state.push_back(tmp);
		tuning_state.push_back(tmp);
		tuning_state.push_back(tmp);


		//set twiddle total
		twiddle_tol = 0.1;

		// set the twiddel flag to True
		twiddle = true;

	}

	cout<<"twiddle = "<<twiddle<<endl;

	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(double cte) {

	static bool first_run =  true;
	static int i = 0;

	if(first_run)
	{
		best_error = cte;
		p_error = cte;
		first_run = false;
	}


	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	if(twiddle == true)
	{

		double temp_dp_sum = dp[0] + dp[1]+dp[2] ;

		if(temp_dp_sum > twiddle_tol)
		{

			switch(tuning_state[i])
			{

			case STEP_UP:

				p[i] += dp[i];

				tuning_state[i] = VALIDATE_STEP_UP;

				break;

			case VALIDATE_STEP_UP:

				if(cte < best_error )
				{
					best_error = cte;

					dp[i] *= 1.1;

					tuning_state[i] = STEP_UP;

				}
				else
				{
					tuning_state[i] = STEP_DOWN;

				}

				break;

			case STEP_DOWN:

				p[i] -= 2 * dp[i];

				tuning_state[i] = VALIDATE_STEP_DOWN;

				break;

			case VALIDATE_STEP_DOWN:

				if(cte < best_error)
				{
					best_error = cte;

					dp[i] *= 1.1;


				}
				else
				{
					p[i] += dp[i];
					dp[i] *= 0.9;

				}

				tuning_state[i] = STEP_UP;
				break;


			}

			// assign the gains of the tuning round
			this->Kp = p[0];
			this->Kd = p[1];
			this->Ki = p[2];

			//check that the i not go out of vector size
			if(i >= dp.size())
			{
				i = 0;
			}

			cout<<"counter value = "<<i<<endl;
		}
		else
		{
			cout<< "Tuning is done" <<endl;

			cout << "Tuned Kp = " << Kp <<endl;
			cout << "Tuned Kd = " << Kd <<endl;
			cout << "Tuned Ki = " << Ki <<endl;

		}


	}
	else
	{


	}

}

double PID::TotalError() {

	return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}

