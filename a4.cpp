
#include "a4.hpp"
#include "util.h"
#include <cmath>

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

  double base_pos = 0.01;
  double base_rot = 0.001;
  
  // Velocity-dependent terms
  double vel_scale = 0.1 * fabs(v);
  double ang_scale = 0.05 * fabs(w);
  
  WMWt[0][0] = (base_pos + vel_scale) * dt * dt;
  WMWt[1][1] = (base_pos + vel_scale) * dt * dt;
  WMWt[2][2] = (base_pos * 2.0 + vel_scale) * dt * dt; 
  WMWt[3][3] = base_rot * dt * dt;
  WMWt[4][4] = base_rot * dt * dt;
  WMWt[5][5] = (base_rot * 5.0 + ang_scale) * dt * dt; 
  
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


  //GPS measurement noise

  R[0][0] = 0.5;
  R[1][1] = 0.5;
  R[2][2] = 1.0;

  //IMU measurement noise
  R[3][3] = 0.01;
  R[4][4] = 0.01;
  R[5][5] = 0.02;

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

  double x = state_in.x[ State::POS_X ], y = state_in.x[ State::POS_Y ], z = state_in.x[ State::POS_Z ];
  double roll = state_in.x[ State::ROT_R ], pitch = state_in.x[ State::ROT_P ], yaw = state_in.x[ State::ROT_Y ];
  
  state_out.x[ State::POS_X ] = x + v * dt * cos( yaw ) * cos( pitch );
  state_out.x[ State::POS_Y ] = y + v * dt * sin( yaw ) * cos( pitch );
  state_out.x[ State::POS_Z ] = z - v * dt * sin( pitch );

  state_out.x[ State::ROT_R ] = roll;
  state_out.x[ State::ROT_P ] = pitch;
  state_out.x[ State::ROT_Y ] = yaw + w * dt;

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

  double yaw = state.x[ State::ROT_Y ];
  double pitch = state.x[ State::ROT_P ];

  A[0][0] = 1.0;
  A[0][4] = -v * dt * cos( yaw ) * sin( pitch );
  A[0][5] = -v * dt * sin( yaw ) * cos( pitch );

  A[1][1] = 1.0;
  A[1][4] = -v * dt * sin( yaw ) * sin( pitch );
  A[1][5] = v * dt * cos( yaw ) * cos( pitch ); 

  A[2][2] = 1.0;
  A[2][4] = -v * dt * cos( pitch );

  A[3][3] = 1.0;
  A[4][4] = 1.0;
  A[5][5] = 1.0;
  
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
  nsf.latitude = state.x[ State::POS_X ];
  nsf.longitude = state.x[ State::POS_Y ];
  nsf.altitude = state.x[ State::POS_Z ];

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
  
  rpy.roll = state.x[ State::ROT_R ];
  rpy.pitch = state.x[ State::ROT_P ];
  rpy.yaw = state.x[ State::ROT_Y ];
  
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

  Hgps[0][0] = 1.0; Hgps[0][1] = 0.0; Hgps[0][2] = 0.0;
  Hgps[1][0] = 0.0; Hgps[1][1] = 1.0; Hgps[1][2] = 0.0;
  Hgps[2][0] = 0.0; Hgps[2][1] = 0.0; Hgps[2][2] = 1.0;

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

  Himu[0][0] = 1.0; Himu[0][1] = 0.0; Himu[0][2] = 0.0;
  Himu[1][0] = 0.0; Himu[1][1] = 1.0; Himu[1][2] = 0.0;
  Himu[2][0] = 0.0; Himu[2][1] = 0.0; Himu[2][2] = 1.0;

}

