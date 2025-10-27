#include "a3.hpp"
#include <cstdlib>
#include <algorithm>
#include <moveit/planning_scene/planning_scene.h>

ASBRContext::ASBRContext( const moveit::core::RobotModelConstPtr& robotmodel,
			  const std::string& name, 
			  const std::string& group ):
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

ASBRContext::~ASBRContext(){}

bool ASBRContext::state_collides( const vertex& q ) const {

  // create a robot state
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "ur5e", q );

  std::vector<double> qr(5, 0.0);
  robotstate.setJointGroupPositions( "robotiq", qr );
  
  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
  
}

ASBRContext::vertex ASBRContext::interpolate( const ASBRContext::vertex& qA, 
					      const ASBRContext::vertex& qB, 
					      double t ){

  ASBRContext::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;

}



ASBRContext::index ASBRContext::select_config_from_tree( const std::vector<ASBRContext::weight>& w ){

  /*
    1. measure total exploration weight
    2, randomly select a value r in [0, total_weight]
    3. iterate through weights, accumulating cumulative_weight
    4. when cumulative_weight >= r, return the current index --> choosing an under-explored region
  
  */

  double total_weight = 0.0;

  for (auto weight :w ) {
    total_weight += weight;
  }

  double r = ((double) rand() / (RAND_MAX)) * total_weight;

  double cumulative_weight = 0.0;

  for (ASBRContext::index i = 0; i < w.size(); i++) {
    cumulative_weight += w[i];

    if (r <= cumulative_weight) {
      return i;
    }
  }
  
  return w.size() - 1; 
}


// TODO
ASBRContext::vertex ASBRContext::sample_nearby( const ASBRContext::vertex& q ){

  /*
  1. define a radius for sampling
  2. for each joint in q, add a random offset within [-radius, radius]
  3. return the new configuration
  */

  ASBRContext::vertex q_rand = q;

  double radius = 0.5; // guessed radius; could be anything

  for (std::size_t i = 0; i < q_rand.size(); i++) {
    double rand_offset = ((double) rand() / RAND_MAX) * (2 * radius) - radius;
    q_rand[i] += rand_offset;
  }

  
  return q_rand;
}

bool ASBRContext::is_local_path_collision_free( const ASBRContext::vertex& q, const ASBRContext::vertex& q_rand ){
  /*
    1. define number of interpolation steps --> how closesly to check for collisions
    2. for each step, interpolate between q and q_rand
    3. check for collision at each interpolated configuration
    4. if any configuration collides, return false; otherwise return true
  */

  int num_steps = 10; // number of interpolation steps

  for (int i = 0; i <= num_steps; i++) {
    double t = (double)(i) / num_steps;
    ASBRContext::vertex q_interp = interpolate(q, q_rand, t);

    if (state_collides(q_interp)) {
      return false; 
    }
  }
  return true;
}


ASBRContext::path ASBRContext::search_path( const std::vector<vertex>& V, const std::vector<index> parent, const index& idx_init, const index& idx_goal ){
  
  /*
  1. initialize an empty path
  2. start from idx_goal and trace back to idx_init using the parent array
  3. push each configuration onto the path
  4. reverse the path to get it from idx_init to idx_goal
  */
  ASBRContext::path P;
  

  index current = idx_goal;

  while (current != idx_init) {
    P.push_back(V[current]);
    current = parent[current];
  }

  P.push_back(V[idx_init]);

  std::reverse(P.begin(), P.end());
  return P;
}

ASBRContext::index ASBRContext::find_nearest_configuration( const std::vector<ASBRContext::vertex>& V, const ASBRContext::vertex& q ){

  /*
  1. initialize minimun
  2. for each configuration in V, compute distance to q
  3. if distance is less than minimum, update minimum and store index
  4. return index of nearest configuration
  */
  
  int imin = 0;

  double min_dist = std::numeric_limits<double>::max();

  for (ASBRContext::index i = 0; i < V.size(); i++) {
    double dist = 0.0;

    for (std::size_t j = 0; j < q.size(); j++) {
      double diff = V[i][j] - q[j];
      dist += diff * diff;
    }

    if (dist < min_dist) {
      min_dist = dist;
      imin = i;
    }
  }
  
  return imin;
}

// TODO
ASBRContext::path ASBRContext::est( const ASBRContext::vertex& q_init, const ASBRContext::vertex& q_goal ){
  ASBRContext::path P;
  

  std::vector<vertex> treeA, treeB;
  std::vector<index> parentA, parentB;
  std::vector<weight> weightsA, weightsB;

  // tree A initialization
  treeA.push_back(q_init);
  parentA.push_back(0);
  weightsA.push_back(1.0);

  // tree B initialization
  treeB.push_back(q_goal);
  parentB.push_back(0);
  weightsB.push_back(1.0);

  const int max_iterations = 10000;

  for (int it = 0; it <max_iterations; it++) {
    std::vector<vertex>& tree_from = (it % 2 == 0) ? treeA : treeB;
    std::vector<vertex>& tree_to = (it % 2 == 0) ? treeB : treeA;
    std::vector<index>& parent_to = (it % 2 == 0) ? parentB : parentA;
    std::vector<weight>& weights_from = (it % 2 == 0) ? weightsA : weightsB;
    std::vector<weight>& weights_to = (it % 2 == 0) ? weightsB : weightsA;

    index idx_from = select_config_from_tree(weights_from);
    vertex q_from = tree_from[idx_from];

    vertex q_rand = sample_nearby(q_from);

    if (!state_collides(q_rand)) {
      index idx_near = find_nearest_configuration(tree_to, q_rand);
      vertex q_near = tree_to[idx_near];

      if (is_local_path_collision_free(q_near, q_rand)) {
        // add to tree
        tree_to.push_back(q_rand);
        parent_to.push_back(idx_near);
        weights_to.push_back(1.0); 

        // check for connecting the two trees...?

        if (is_local_path_collision_free(q_rand, q_from)) {
          path p1 = search_path(treeA, parentA, 0, treeA.size() - 1);
          path p2 = search_path(treeB, parentB, 0, treeB.size() - 1);

          std::reverse(p2.begin(), p2.end());

          p1.insert(p1.end(), p2.begin(), p2.end());
          return p1;
        }
        
      }
    }
  }

  return ASBRContext::path(); // empty path
}

bool ASBRContext::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
  							      getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;

  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  rclcpp::Clock clock;
  rclcpp::Time t1 = clock.now();
  path P = est( qstart, qfinal );
  rclcpp::Time t2 = clock.now();
  std::cout << "Your path has length " << P.size() << std::endl;
  // end the timer

  // The rest is to fill in the animation.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", qstart );
  res.trajectory_->addSuffixWayPoint( robotstate, 0.0001 );

  for( std::size_t i=1; i<P.size(); i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i-1], P[i], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.0001 );
    }
  }

  //
  rclcpp::Duration planning_time = t2-t1;
  res.planning_time_ = planning_time.seconds();
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool ASBRContext::solve( planning_interface::MotionPlanDetailedResponse& )
{ return true; }

void ASBRContext::clear(){}

bool ASBRContext::terminate(){return true;}

















/*

#include "assignment3_context.hpp"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

class ASBRPlannerManager : public planning_interface::PlannerManager{

public:

  ASBRPlannerManager() : planning_interface::PlannerManager() {}

  virtual 
  bool 
  initialize
  ( const moveit::core::RobotModelConstPtr& model,
    const rclcpp::Node::SharedPtr&,
    const std::string&){
    context.reset( new ASBRContext( model, 
				    std::string( "ASBR" ),
				    std::string( "manipulator" ) ) );
    
    return true;
  }

  virtual
  bool 
  canServiceRequest
  ( const moveit_msgs::msg::MotionPlanRequest& ) const 
  { return true; }

  virtual std::string getDescription() const
  { return std::string( "ASBRPlanner" ); }

  virtual
  void 
  getPlanningAlgorithms
  ( std::vector<std::string> &algs ) const{
    algs.resize(1);
    algs[0] = "ASBRPlanner";
  }

  virtual 
  planning_interface::PlanningContextPtr 
  getPlanningContext
  ( const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes&) const{
    context->setPlanningScene( planning_scene );
    context->setMotionPlanRequest( req );
    return planning_interface::PlanningContextPtr( context );
  }

  virtual 
  void 
  setPlannerConfigurations
  (const planning_interface::PlannerConfigurationMap&){}

private:
  
  std::shared_ptr<ASBRContext> context;

};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( ASBRPlannerManager, planning_interface::PlannerManager );


*/