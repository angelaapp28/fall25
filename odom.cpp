#include <math.h>
#include <stdlib.h>
#include <iostream>

/**
   TODO
   Implement a sample motion odometry
   \input sample: The pose of a sample
                  sample[0] is the x coordinate
		  sample[1] is the y coordinate and
		  sample[2] is the angle.
   \output sample: The updated sample
   \input delta: The pose increment in world frame
                 delta[0] = x_t - x_(t-1)
                 delta[1] = y_t - y_(t-1)
                 delta[2] = theta_t - theta_(t-1)
   \input alpha1: robot error parameter
   \input alpha2: robot error parameter
   \input alpha3: robot error parameter
   \input alpha4: robot error parameter
*/

void odometry_model( double sample[3],
		     double delta[3],
		     double alpha1,
		     double alpha2,
		     double alpha3,
		     double alpha4 ){

  double x = sample[0];
  double y = sample[1];
  double theta = sample[2];

  double delta_x = delta[0];
  double delta_y = delta[1];
  double delta_theta = delta[2];

  double delta_rot1 = atan2( delta_y, delta_x ) - theta;
  double delta_trans = sqrt( delta_x * delta_x + delta_y * delta_y);
  double delta_rot2 = delta_theta - delta_rot1;

  // noisify
  double delta_rot1_hat = delta_rot1 - pf_ran_gaussian(alpha1 * pow(delta_rot1, 2) + alpha2 * pow(delta_trans, 2));
  double delta_trans_hat = delta_trans - pf_ran_gaussian(alpha3 * pow(delta_trans, 2) + alpha4 * pow(delta_rot1, 2) + alpha4 * pow(delta_rot2, 2));
  double delta_rot2_hat = delta_rot2 - pf_rand_gaussian(alpha1 * pow(delta_rot2, 2) + alpha2 * pow(delta_trans, 2));

  double x_prime = x + delta_trans_hat * cos(theta + delta_rot1_hat);
  double y_prime = y + delta_trans_hat * sin(theta + delta_rot1_hat);
  double theta_prime = theta + delta_rot1_hat + delta_rot2_hat;

  sample[0] = x_prime;
  sample[1] = y_prime;
  sample[2] = theta_prime;

  // normalize angle
   while (sample[2] > M_PI) sample[2] -= 2.0 *M_PI;
   while (sample[2] < -M_PI) sample[2] += 2.0 *M_PI;
}

