#ifndef PID_H
#define PID_H
#include <vector>

typedef enum
{
	STEP_UP,
	STEP_DOWN,
	VALIDATE_STEP_UP,
	VALIDATE_STEP_DOWN
}gain_state;

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

  /*vector of P*/
  std::vector<double> p;
  /*vector of dp*/
  std::vector<double> dp;

  /*twiddle flag*/
  bool twiddle;

  /*twiddle total*/
  double twiddle_tol;

  /*twiddle gain tuning state*/
  std::vector<gain_state> tuning_state;


  /*best error*/
  double best_error;

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
};

#endif /* PID_H */
