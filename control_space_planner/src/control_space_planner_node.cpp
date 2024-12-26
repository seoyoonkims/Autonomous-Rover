#include "control_space_planner/control_space_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  subEgoOdom = nh.subscribe("/odom",1, &MotionPlanner::CallbackEgoOdom, this);
  // subGoalPoint = nh.subscribe("/move_base_simple/goal",1, &MotionPlanner::CallbackGoalPoint, this);
  
  sub_global_path = nh.subscribe("/graph_planner/path/global_path", 1, &MotionPlanner::CallbackGoalPoint, this);
  sub_new_graph = nh.subscribe("/bool/graph_planner/done", 1, &MotionPlanner::Receive, this);
  
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubTruncTarget = nh_.advertise<geometry_msgs::PoseStamped>("/car/trunc_target", 1, true);
  pubMotionPlan = nh_.advertise<nav_msgs::Path>("/path/selected_motion", 1, true);
  
  
};


bool trigger = false;
void MotionPlanner::Receive(const std_msgs::Bool& msg)
{
  trigger = msg.data;
}
MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  this->localMap = msg;
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  this->frame_id = msg.header.frame_id;
  this->mapResol = msg.info.resolution;
  bGetMap = true;
}

bool visited[100];
double distance;
int i;

void MotionPlanner::CallbackGoalPoint(const nav_msgs::Path& msg)
{
  if (trigger){
    for (int j = 0;j < 100; ++j) {
        visited[j] = false;
    }
    i = 0;
  }
  else {
    distance = std::sqrt(std::pow(this->ego_x - msg.poses[i].pose.position.x, 2) + 
                              std::pow(this->ego_y - msg.poses[i].pose.position.y, 2));
                              
    if (!visited[i] && distance < 0.15){
      visited[i] = true;
      i++;
    }
  }

  this->goalPose = msg.poses[i];
  this->goal_x = this->goalPose.pose.position.x;
  this->goal_y = this->goalPose.pose.position.y;
  
  // this->goalPose = msg.poses[0];
  // - position
  // this->goal_x = msg.pose.position.x;
  // this->goal_y = msg.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion goal;
  double goal_roll, goal_pitch, goal_yaw;
  // --- copy quaternion from odom
  
  tf2::convert(this->goalPose.pose.orientation, goal);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(goal);
  m_goal.getRPY(goal_roll, goal_pitch, goal_yaw);
  this->goal_yaw = goal_yaw;
  
  this->bGetGoal = true;
}

void MotionPlanner::CallbackEgoOdom(const nav_msgs::Odometry& msg)
{
  this->egoOdom = msg;
  // - position
  this->ego_x = msg.pose.pose.position.x;
  this->ego_y = msg.pose.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion ego;
  double ego_roll, ego_pitch, ego_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.pose.orientation, ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_ego(ego);
  m_ego.getRPY(ego_roll, ego_pitch, ego_yaw);
  this->ego_yaw = ego_yaw;
  
  this->bGetEgoOdom = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  nav_msgs::Path minCostPath;

  // publish selected motion primitive as point cloud
  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);

    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.pose.position.x = pointTmp.x;
    tmp_pose.pose.position.y = pointTmp.y;
    tmp_pose.pose.position.z = 0.0;
    tmp_pose.pose.orientation.w = 1.0;
    tmp_pose.pose.orientation.x = 0.0;
    tmp_pose.pose.orientation.y = 0.0;
    tmp_pose.pose.orientation.z = 0.0;
    minCostPath.poses.push_back(tmp_pose);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);

  minCostPath.header.frame_id = this->frame_id;
  minCostPath.header.stamp = ros::Time::now();
  pubMotionPlan.publish(minCostPath);

}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish motion primitives as point cloud
  for (auto& motionPrimitive : motionPrimitives) {
    double cost_total = motionPrimitive.back().cost_total;
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      pointTmp.z = cost_total;
      pointTmp.intensity = cost_total;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{

  geometry_msgs::Twist command;
  // low-level control
  double steering_angle = motionMinCost.back().delta;
  double yaw = motionMinCost.back().yaw;
  double speed_norm = 0.1 * this->MOTION_VEL * motionMinCost.size() / (this->MAX_PROGRESS / this->DIST_RESOL);
  
  command.angular.z = speed_norm * tan(steering_angle) / this->WHEELBASE * 0.5;
  command.linear.x  = speed_norm * cos(yaw);
  command.linear.y  = speed_norm * sin(yaw);

  // arrival rule
  if (bGetGoal && bGetLocalNode) {
    double distToGoal = sqrt(this->localNode.x*this->localNode.x + this->localNode.y*this->localNode.y);

    if (distToGoal < this->ARRIVAL_THRES) {
      command.angular.z = 0.0;
      command.linear.x = 0.0;
      command.linear.y = 0.0; // modify
    }
    else if (distToGoal < 2*this->ARRIVAL_THRES) {
      command.linear.x = command.linear.x * pow(distToGoal / 2*this->ARRIVAL_THRES, 3.0);
      command.linear.y = command.linear.y * pow(distToGoal / 2*this->ARRIVAL_THRES, 3.0); // modify
    }
  }

  pubCommand.publish(command);
}

/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Compute current LOS target pose
  if (this->bGetEgoOdom && this->bGetGoal) {
    Node goalNode;
    goalNode.x = this->goal_x;
    goalNode.y = this->goal_y;
    goalNode.yaw = this->goal_yaw;
    localNode = GlobalToLocalCoordinate(goalNode, this->egoOdom);
    // - compute truncated local node pose within local map
    Node tmpLocalNode;
    memcpy(&tmpLocalNode, &localNode, sizeof(struct Node));
    tmpLocalNode.x = std::max(this->mapMinX, std::min(tmpLocalNode.x, this->mapMaxX));
    tmpLocalNode.y = std::max(this->mapMinY, std::min(tmpLocalNode.y, this->mapMaxY));
    truncLocalNode = tmpLocalNode;

    // for debug
    geometry_msgs::PoseStamped localPose = GlobalToLocalCoordinate(this->goalPose, this->egoOdom);
    localPose.header.frame_id = "base_link";
    pubTruncTarget.publish(localPose);

    this->bGetLocalNode = true;
  }

  // Motion generation
  motionCandidates = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionCandidates);

  // Publish data
  PublishData(motionMinCost, motionCandidates);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */


  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;

  // compute params w.r.t. uncertainty
  int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // the maximum displacement / resolution * 2 for considering both left/right direction

  // max progress of each motion
  double maxProgress = this->MAX_PROGRESS; // the maximum distance of progress
  for (int i=0; i<num_candidates+1; i++) {
    // current steering angle_delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

    // initialize start node
    Node startNode(0, 0, 0, 0, angle_delta, 0, 0, 0, -1, false);
    
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, this->localMap);

    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }

  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: rollout to generate a motion primitive based on the current steering angle
    - calculate cost terms here if you need
    - check collision / sensor range if you need
    1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
    2. collision checking
    3. range checking
  */

  // Initialize motionPrimitive
  std::vector<Node> motionPrimitive;

  // Check collision and compute traversability cost for each motion node of primitive (in planner coordinate)
  // Initialize current motion node
  // Node(double x, double y, double z, double yaw, double delta, double cost_control, double cost_colli, double cost_total, int idx, bool collision)
  // modify : intial idx = 0 instead of -1
  Node currMotionNode(startNode.x, startNode.y, 0, 0, startNode.delta, 0, 0, 0, 0, false);
  double progress = this->DIST_RESOL;

  // for compute closest distance toward goal point. You can use in SelectMotion function to calculate goal distance cost
  double minDistGoal = 987654321;
  if (this->bGetLocalNode) {
    minDistGoal = sqrt((startNode.x-truncLocalNode.x)*(startNode.x-truncLocalNode.x) +
                       (startNode.y-truncLocalNode.y)*(startNode.y-truncLocalNode.y));
  }
  
  //! 1. Update motion node using current steering angle delta based on the vehicle kinematics equation
  // - while loop until maximum progress of a motion

  // Loop for rollout
  while (progress < maxProgress) {
    // Calculate minimum distance toward goal
    if (this->bGetLocalNode) {
      double distGoal = sqrt((currMotionNode.x-truncLocalNode.x)*(currMotionNode.x-truncLocalNode.x) +
                             (currMotionNode.y-truncLocalNode.y)*(currMotionNode.y-truncLocalNode.y));
      if (minDistGoal > distGoal) {
        minDistGoal = distGoal;
      }
    }
    currMotionNode.minDistGoal = minDistGoal; // save current minDistGoal at current node
    
    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    // collision/range checking with "lookahead" concept
    double aheadX = currMotionNode.x + this->MOTION_VEL * cos(currMotionNode.yaw) * this->TIME_RESOL;
    double aheadY = currMotionNode.y + this->MOTION_VEL * sin(currMotionNode.yaw) * this->TIME_RESOL;
    double aheadYaw = currMotionNode.yaw + this->MOTION_VEL * tan(startNode.delta) / this->WHEELBASE * this->TIME_RESOL;
    
    //! 2. collision checking
    // - local to map coordinate transform
    Node collisionPointNode(aheadX, aheadY, 0, aheadYaw, currMotionNode.delta, 0, 0, 0, currMotionNode.idx + 1, false);
    Node collisionPointNodeMap = LocalToPlannerCorrdinate(collisionPointNode);
    if (CheckCollision(collisionPointNodeMap, localMap)) {
      // - do some process when collision occurs.
      // - you can save collision information & calculate collision cost here.
      // - you can break and return current motion primitive or keep generating rollout.
      currMotionNode.collision = true;
      currMotionNode.cost_colli = 1 / (this->mindist_colli + std::numeric_limits<double>::epsilon());
      //currMotionNode.cost_colli = 10;
      motionPrimitive.push_back(currMotionNode);
      return motionPrimitive;
    }

    //! 3. range checking
    // - if you want to filter out motion points out of the sensor range, calculate the Line-Of-Sight (LOS) distance & yaw angle of the node
    // - LOS distance := sqrt(x^2 + y^2)
    // - LOS yaw := atan2(y, x)
    // - if LOS distance > MAX_SENSOR_RANGE or abs(LOS_yaw) > FOV*0.5 <-- outside of sensor range 
    // - if LOS distance <= MAX_SENSOR_RANGE and abs(LOS_yaw) <= FOV*0.5 <-- inside of sensor range
    // - use params in header file (MAX_SENSOR_RANGE, FOV)
    double LOS_DIST = sqrt((collisionPointNode.x - startNode.x)*(collisionPointNode.x - startNode.x)+(collisionPointNode.y - startNode.y)*(collisionPointNode.y - startNode.y));
    double LOS_YAW = atan2(collisionPointNode.y - startNode.y, collisionPointNode.x - startNode.x);
    if (LOS_DIST > this->MAX_SENSOR_RANGE || abs(LOS_YAW) > this->FOV*0.5) {
      // -- do some process when out-of-range occurs.
      // -- you can break and return current motion primitive or keep generate rollout.      
      motionPrimitive.push_back(currMotionNode);
      return motionPrimitive;
    }

    // append collision-free and out-of-range-free motion in the current motionPrimitive
    motionPrimitive.push_back(currMotionNode);

    // update progress of motion
    progress += this->DIST_RESOL;
    
    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    currMotionNode.x = aheadX;
    currMotionNode.y = aheadY;
    currMotionNode.yaw = aheadYaw;
    currMotionNode.idx += 1;
    
  }
  
  // return current motion
  return motionPrimitive;
}


std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  /*
    TODO: select the minimum cost motion primitive
  
    1. Calculate cost terms
    2. Calculate total cost (weighted sum of all cost terms)
    3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
    4. Return minimum cost motion
  */

  double minCost = 9999999;
  std::vector<Node> motionMinCost; // initialize as odom

  // check size of motion primitives
  if (motionPrimitives.size() != 0) {
    // Iterate all motion primitive (motionPrimitive) in motionPrimitives
    for (auto& motionPrimitive : motionPrimitives) {
      //!1. Calculate cost terms
      // Node(double x, double y, double z, double yaw, double delta, double cost_control, double cost_colli, double cost_total, int idx, bool collision)
      Node lastNode = motionPrimitive[motionPrimitive.size() - 1];
      //printf("x: %f, y: %f, yaw: %f, delta: %f, cost_control: %f, cost_colli: %f, cost_total: %f, idx: %d, bool: %s\n", lastNode.x, lastNode.y, lastNode.yaw, lastNode.delta, lastNode.cost_control, lastNode.cost_colli, lastNode.cost_total, lastNode.idx, lastNode.collision ? "true" : "false");
      double w_colli = 25.0 * 1.0;// TODO
      double w_distance = 100.0 * 10.0;// TODO
      //double w_direction = 100.0 * 0.01;// TODO
      double w_progress = 1.0 * 1.0;// TODO
      double w_steering = 75.0 * 10.0;// TODO
      
      double cost_colli = 0.0;
      if (lastNode.collision){
        cost_colli = lastNode.cost_colli;
      }
      double goal_distance = lastNode.minDistGoal;
      //double goal_direction = atan2(abs(lastNode.y-truncLocalNode.y), abs(lastNode.x-truncLocalNode.x));
      double cost_progress = - (double)(lastNode.idx);
      double cost_steering = abs(lastNode.delta);
      
      //! 2. Calculate total cost ex) collision cost, goal distance, goal direction, progress cost, steering cost....
      //printf("colli: %f, dist: %f, direc: %f, progress: %f, steering: %f\n", w_colli * cost_colli, w_distance * goal_distance, w_direction * goal_direction, w_progress * cost_progress, w_steering * cost_steering);
      //double cost_total = w_colli * cost_colli + w_distance * goal_distance + w_direction * goal_direction + w_progress * cost_progress + w_steering * cost_steering;
      double cost_total = w_colli * cost_colli + w_distance * goal_distance + w_progress * cost_progress + w_steering * cost_steering;
      
      //! 3. Compare & Find minimum cost & minimum cost motion
      if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
      }
    }
  }
  //! 4. Return minimum cost motion
  return motionMinCost;
}

/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node goalNodePlanner, nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: check collision of the current node
    - the position x of the node should be in a range of [0, map width]
    - the position y of the node should be in a range of [0, map height]
    - check all map values within the inflation area of the current node
    - use params in header file: INFLATION_SIZE, OCCUPANCY_THRES
  */
  double tmp_x = 0;
  double tmp_y = 0;
  int map_index = 0;
  int16_t map_value = 0;
  int map_width = std::ceil((this->mapMaxX - this->mapMinX) / this->mapResol);
  int map_height = std::ceil((this->mapMaxY - this->mapMinY) / this->mapResol);
  this->mindist_colli = 9999999; // the minimum distance to obstacle to collide with
  bool colli = false;

  if (goalNodePlanner.x < 0 || goalNodePlanner.x > map_width){
    this->mindist_colli = this->mapResol * 0.1;
    return true;
  }
  if (goalNodePlanner.y < 0 || goalNodePlanner.y > map_height){
    this->mindist_colli = this->mapResol * 0.1;
    return true;
  }
  for (int i = 0; i < this->INFLATION_SIZE; i++){
    for (int j = 0; j < this->INFLATION_SIZE; j++){
      tmp_x = goalNodePlanner.x + i - 0.5 * this->INFLATION_SIZE; // you need to check whether this tmp_x is in [0, map width]
      tmp_y = goalNodePlanner.y + j - 0.5 * this->INFLATION_SIZE; // you need to check whether this tmp_y is in [0, map height]
      
      if (tmp_x < 0 || tmp_x > map_width){
        this->mindist_colli = this->mapResol * 0.1;
        return true;
      }
      if (tmp_y < 0 || tmp_y > map_height){
        this->mindist_colli = this->mapResol * 0.1;
        return true;
      }
      
      map_index = (int)(map_width * (int)(tmp_y) + (int)(tmp_x));
     
      //printf("map_width: %d, map_height: %d, tmp_y: %f, tmp_x: %f, map_index: %d, mapsize: %ld\n", map_width, map_height, tmp_y, tmp_x, map_index, localMap.data.size());
       
      if (map_index >= 0 && map_index <= localMap.data.size()-1){
        map_value = static_cast<int16_t>(localMap.data[map_index]);
        if (map_value > this->OCCUPANCY_THRES || map_value < 0){
          double dist_colli = this->mapResol * sqrt((tmp_x-goalNodePlanner.x)*(tmp_x-goalNodePlanner.x) + (tmp_y-goalNodePlanner.y)*(tmp_y-goalNodePlanner.y));
          if (this->mindist_colli > dist_colli){
            this->mindist_colli = dist_colli;
          }
          colli = true;
          //return true;
        }
      }
      else{
        this->mindist_colli = this->mapResol * 0.1;
        return true;
      }
    }
  }
  
  if (colli){
    return true;
  }
  return false;
  
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap && this->bGetGoal) {
  // if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << "bGetMap : " << bGetMap << " bGetGoal : " << bGetGoal << std::endl;
    return false;
  }
}

Node MotionPlanner::GlobalToLocalCoordinate(Node globalNode, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  Node tmpLocalNode;
  // - Copy data globalNode to tmpLocalNode
  memcpy(&tmpLocalNode, &globalNode, sizeof(struct Node));

  // - Coordinate transform
  // -- translatioonal transform
  double delX = globalNode.x - egoOdom.pose.pose.position.x;
  double delY = globalNode.y - egoOdom.pose.pose.position.y;
  double delZ = globalNode.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_ego;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_odom(q_ego);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = globalNode.yaw - egoY;

  // - Update pose
  tmpLocalNode.x = newX;
  tmpLocalNode.y = newY;
  tmpLocalNode.z = newZ;
  tmpLocalNode.yaw = newYaw;

  return tmpLocalNode;
}

geometry_msgs::PoseStamped MotionPlanner::GlobalToLocalCoordinate(geometry_msgs::PoseStamped poseGlobal, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  // - Copy data nodeGlobal to nodeLocal
  geometry_msgs::PoseStamped poseLocal;

  // - Coordinate transform
  // -- translatioonal transform
  double delX = poseGlobal.pose.position.x - egoOdom.pose.pose.position.x;
  double delY = poseGlobal.pose.position.y - egoOdom.pose.pose.position.y;
  double delZ = poseGlobal.pose.position.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_goal, q_ego;
  double goalR, goalP, goalY;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(poseGlobal.pose.orientation, q_goal);
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(q_goal);
  tf2::Matrix3x3 m_odom(q_ego);
  m_goal.getRPY(goalR, goalP, goalY);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = goalY - egoY;

  // - Update pose
  // -- quaternion to RPY 
  // -- RPY to Quaternion
  tf2::Quaternion globalQ, globalQ_new;
  globalQ.setRPY(0.0, 0.0, newYaw);
  globalQ_new = globalQ.normalize();

  poseLocal.pose.position.x = newX;
  poseLocal.pose.position.y = newY;
  poseLocal.pose.position.z = newZ;
  tf2::convert(globalQ_new, poseLocal.pose.orientation);

  return poseLocal;
}

Node MotionPlanner::LocalToPlannerCorrdinate(Node nodeLocal)
{
  /*
    TODO: Transform from local to occupancy grid map coordinate
    - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
    - map coordinate ([cell]): x [0, map width], y [map height]
    - convert [m] to [cell] using map resolution ([m]/[cell])
  */
  // Copy data nodeLocal to nodeMap
  Node nodeMap;
  memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));
  // Transform from local (min x, max x) [m] to map (0, map width) [grid] coordinate
  nodeMap.x = (nodeLocal.x - this->mapMinX) / this->mapResol;
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  nodeMap.y = (nodeLocal.y - this->mapMinY) / this->mapResol;

  return nodeMap;
}


/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);
  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);
  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "control_space_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
