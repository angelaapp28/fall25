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

  // TODO
}

