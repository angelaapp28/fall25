#include <cmath>

double gaussian(double x, double mu, double sigma){
    return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp( -0.5 * pow((x - mu) / sigma, 2));
}

/**
  Correct range: p_hit
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \input: sigma_hit: sensor standart deviation
  \return phit
*/
double p_hit( double r, double rs, double z_max, double sigma_hit ){
  double phit = 0.0;

  if (r < 0 || r > z_max || std::isnan(r) || std::isinf(r)){
      return 0.0;
  }
  double g = gaussian(r, rs, sigma_hit);
    
  double eta = 1.0 / (erf((z_max - rs) / (sigma_hit * sqrt(2))) - erf(-rs / (sigma_hit * sqrt(2)))) * 0.5;
    
  phit = eta * g;
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
  
  double pshort = 0.0;
  if (r < 0 || r > z_max || std::isnan(r) || std::isinf(r)){
      return 0.0;
  }

  double eta = 1.0 / (1.0 - exp(-lambda_short * rs));
  pshort = eta * lambda_short * exp(-lambda_short * r);
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
  if (r < 0 || r > z_max || std::isnan(r) || std::isinf(r)){
      return 0.0;
  }

  double pmax = 0.0;
  if (r - z_max < 1e-4){
      pmax = 1.0;
  }
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
  double prand = 0;
  if (r < 0 || r > z_max || std::isnan(r) || std::isinf(r)){
      return 0.0;
  }

  prand = 1.0 / z_max;
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
  double robot_x, robot_y, robot_theta = s;
  double laser_x, laser_y, laser_theta = laser_pose;

  // transform laser to world (robot * laser -- matrix multiplication)
  double laser_world_x = robot_x + (laser_x * cos(robot_theta))  - (laser_y * sin(robot_theta));
  double laser_world_y = robot_y + (laser_x * sin(robot_theta))  + (laser_y * cos(robot_theta));
  double laser_world_theta = robot_theta + laser_theta;

  for (size_t i = 0; i < z.size(); i++){
    double measured_range = z[i].range;
    double measured_bearing = z[i].bearing;

    //  beam endpoint in world coordinates
    double beam_world_bearing = laser_world_theta + measured_bearing;
    // find expected range
    double expected_range = map_calc_range(map, laser_world_x, laser_world_y, beam_world_bearing, z_max);

    double phit = p_hit(measured_range, expected_range, z_max, sigma_hit);
    double pshort = p_short(measured_range, expected_range, z_max, lambda_short);      double pmax = p_max(measured_range, expected_range, z_max);
    double prand = p_rand(measured_range, expected_range, z_max);

    double pz = w_hit * phit + w_short * pshort + w_max * pmax + w_rand * prand;

    // update total probability
    p += log(pz + 1e-9);
  }
  return p;
}
