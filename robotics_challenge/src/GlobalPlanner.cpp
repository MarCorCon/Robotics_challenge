#include"GlobalPlanner.h"


namespace global_planner {

GlobalPlanner::GlobalPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialized_ = false;
  initialize(name, costmap_ros);

}

void GlobalPlanner::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_){
	costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter. 
	costmap_ = costmap_ros_->getCostmap();       
	initialized_ = true;
  }
     else
       ROS_WARN("This planner has already been initialized... doing nothing");
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector< geometry_msgs::PoseStamped >& plan)
{
  // Planning algorithm to be implemented
  
  ROS_INFO("X: %d", (int)costmap_->getSizeInCellsX());
  ROS_INFO("Y: %d", (int)costmap_->getSizeInCellsY());
  ROS_INFO("%s",costmap_ros_->getBaseFrameID().c_str());
  ROS_INFO("%s",costmap_ros_->getGlobalFrameID().c_str());
 double x,y;
 astar_.clear();
  for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); j++) {
      // Values: 0. 62, 253 and 254
      if (costmap_->getCost(i,j)==0)
      {
	int id0 = i*costmap_->getSizeInCellsY() + j;
	costmap_->mapToWorld(i,j,x,y);
	astar_.addNode(id0 , x, y);
	if (i>0 && costmap_->getCost(i-1,j)==0) {
		int id1 = (i-1)*costmap_->getSizeInCellsY() + j;
		astar_.addEdge(id0 , id1);
	}
	if (j>0 && costmap_->getCost(i,j-1)==0) {
		int id1 = i*costmap_->getSizeInCellsY() + (j-1);
		astar_.addEdge(id0 , id1);
	}
	if (i>0 && j>0 && costmap_->getCost(i-1,j-1)==0) {
		int id1 = (i-1)*costmap_->getSizeInCellsY() + (j-1);
		astar_.addEdge(id0 , id1);
	}
	if (i>0 && j<(costmap_->getSizeInCellsY()-1) && costmap_->getCost(i-1,j+1)==0) {
		int id1 = (i-1)*costmap_->getSizeInCellsY() + (j+1);
		astar_.addEdge(id0 , id1);
	}
      }
    }
  }
 unsigned int start_i, start_j, goal_i, goal_j;
  int start_id, goal_id;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,start_i,start_j);
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,goal_i,goal_j);
  start_id = start_i*costmap_->getSizeInCellsY() + start_j;
  goal_id = goal_i*costmap_->getSizeInCellsY() + goal_j;
  std::list<AStarPoint> path;
  astar_.getPath(start_id,goal_id,path);
  plan.clear();
  for (auto it = path.begin(); it!= path.end(); ++it) {
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = it->x;
	pose.pose.position.y = it->y;
	plan.push_back(pose);
  }
  return true;
}





}
