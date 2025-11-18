
#include "ekf_models.hpp"
#include "utilities.h"

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of C are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] Q Covariance matrix of the system.
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_WMWt( double WMWt[6][6], const State& state, double v, double w, double dt ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      WMWt[r][c] = 0.0;

  // TODO
  // Fill in WMWt
  
}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R( double R[6][6], const State& state ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      R[r][c] = 0.0;

  // TODO fill in the matrix R
}


/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
State sys_evaluate_f( const State& state_in, double v, double w, double dt ){

  State state_out;

  // TODO Given state_in and delta_x and delta_w increments determine the prior
  // estimate state_out
      
  return state_out;
}

/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_A( double A[6][6], const State& state, double v, double w, double dt ){
  
  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      A[r][c] = 0.0;
  
  // TODO
  // Given state, v, w and dt. Compute the system Jacobian G
  
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::msg::NavSatFix meas_evaluate_gps( const State& state ){

  sensor_msgs::msg::NavSatFix nsf;

  // TODO
  // Given prior estimate state, determine the expected GPS measurement nsf

  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::msg::RPY meas_evaluate_imu( const State& state ){
  sensor_msgs::msg::RPY rpy;

  // TODO
  // Given the prior estimate state, determine the expected RPY measurement rpy  
  return rpy;
}

/** 
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state ){

  // TODO
  // Fill the 3x3 Jacobian matrix Hgps of the GPS observations
}

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state ){

  // TODO
  // Fill the 3x3 Jacobian matrix Himu of the IMU observations
}

