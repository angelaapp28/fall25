#include <cmath>

/**
  Correct range: p_hit
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \input: sigma_hit: sensor standart deviation
  \return phit
*/
double p_hit( double r, double rs, double z_max, double sigma_hit ){
    double phit=0.0;

    if (r < 0 || r > z_max){
        return 0.0;
    } else if (r == NAN || r==INFINITY){
        return 0.0;
    }

    double denom = sqrt(2.0 * M_PI * sigma_hit * sigma_hit);
    double num = exp( -0.5 * (r - rs) * (r - rs) / (sigma_hit * sigma_hit) );
    return phit;
}

/**
  Unexpected object: p_short
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input lambda_short: exponential parameter
  \return p_short
*/
double p_short( double r, double rs, double z_max, double lambda_short ){
  // TODO
  double pshort = 0.0;
  return pshort;
}

/**
  Failure: p_max
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \return p_max
 */
double p_max( double r, double rs, double z_max ){
  // TODO
  double pmax = 0.0;
  return pmax;
}  

/**
   Random measurement: p_rand
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \return p_rand
*/
double p_rand( double r, double rs, double z_max ){
  // TODO
  double prand=0;
  return prand;
}

/**
   beam model
   Implement the beam model for a laser range finder. The function
   returns the probability P( z | s, m ), that is the probability that a 
   measurement z was caused by a robot in a state s and in a map m.
   \input Map* m:  The map of the environment the robot.
   \input Scan z: A vector of laser beams. Each beam has two members: range and 
                  bearing. You can access the range if the ith beam with
		  z[i].range and the bearing of the ith beam with z[i].bearing.
   \input State s: The state of the robot with respect to the world frame.
                   The x coordinate is given by s[0], the y coordinate by s[1]
		   and the angle theta by s[2].
   \input Pose laser_pose: The position and orientation of the laser with 
                           respect to the robot's coordinate frame. The x, y
			   and angle are given by coordinate is laser_pose[0]
			   laser_pose[1] and laser_pose[2] respectively.
   \input sigma_hit: The standard variation of errors of a beam
   \input lambda_short: The p_short exponential parameter
   \input z_max:  The laser maximum range (in meters)
   \input w_hit:  The coefficient of measurements
   \input w_short:The coefficient of measurements
   \input w_max:  The coefficient of measurements
   \input w_rand: The coefficient of random errors
   \return        The probability p( z | s, m )
*/

double beam_model( Map* map,
		   Scan& z,
		   State s, 
		   Pose laser_pose, 
		   double sigma_hit, 
		   double lambda_short,
		   double z_max, 
		   double w_hit,
		   double w_short,
		   double w_max,
		   double w_rand ){

  double p = 1.0;
  return p;
}
