#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#define foreach BOOST_FOREACH

double getDistance(const geometry_msgs::PoseStamped::ConstPtr& pose, double x, double y)
{
	return sqrt( (pose->pose.position.x - x) * (pose->pose.position.x -x) + (pose->pose.position.y - y)*(pose->pose.position.y - y) ); 
}


int main(int argc, char** argv)
{
	std::string bag_file;
	ros::init(argc, argv, "metrics");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double start_x, start_y;
	double goal_x, goal_y;
	double last_x,last_y;
	double distance_threshold;
	tf::TransformListener tf;
	
	pn.param<double>("start_x",start_x,3.46);
	pn.param<double>("start_y",start_y,4.62);
	pn.param<double>("distance_threshold",distance_threshold,0.25);
	
	pn.param<double>("goal_x",goal_x,6.0);
	pn.param<double>("goal_y",goal_y,9.0);
	
	pn.param<std::string>("bag_file",bag_file,""); 

	rosbag::Bag bag;
	ROS_INFO("Opening bag %s...",bag_file.c_str());
	bag.open(bag_file,rosbag::bagmode::Read);
	ROS_INFO("Processing bag %s...",bag_file.c_str());	

	std::vector<std::string> topics;
	
	topics.push_back("/map_pose");
	topics.push_back("/scan");
	bool first_pose=true;
	double time=0,distance=0;
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	ros::Time initial_time;
	std::vector<double> laser_mins;
	foreach(rosbag::MessageInstance const m, view) {
		
		if (m.getTopic().compare("/map_pose")==0) {
			geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
			if (first_pose && getDistance(pose,start_x,start_y)>distance_threshold) {
				ROS_ERROR("Invalid initial position: (%f,%f) Distance: %f ",pose->pose.position.x,pose->pose.position.y,  getDistance(pose,start_x,start_y));
				break;
			} else if (getDistance(pose,goal_x,goal_y)<distance_threshold) {
				time = (pose->header.stamp - initial_time).toSec();
				ROS_INFO("Trajectory finished");
				break;
			}
			if (first_pose) {
				initial_time = pose->header.stamp;
				first_pose=false;
			} else {
				distance+=getDistance(pose,last_x,last_y);
			}
			last_x = pose->pose.position.x;
			last_y = pose->pose.position.y;

		} else if (m.getTopic().compare("/scan")==0) {
			
			sensor_msgs::LaserScan::ConstPtr laser = m.instantiate<sensor_msgs::LaserScan>();
			double min = 999999;	
			unsigned counter=0;		
			for (unsigned i = 0; i< laser->ranges.size(); i++) {
				if (!isnan(laser->ranges[i]) && !isinf(laser->ranges[i])) {
					if (laser->ranges[i]<min) {
						min = laser->ranges[i];
					}
					counter++;
				}
			}
			if (counter>0) {
				laser_mins.push_back(min);
			}
		
		}
	
	}
	std::cout<<"TRAJECTORY TIME: "<<time<<" seconds"<<std::endl;
	std::cout<<"TRAJECTORY DISTANCE: "<<distance<<" meters"<<std::endl;
	if (laser_mins.size()>0) {
		double average_value=0;
		for (unsigned i=0;i<laser_mins.size();i++) {
			average_value += laser_mins[i];
		}
		average_value/=(double)laser_mins.size();
		double sd=0;
		for (unsigned i=0;i<laser_mins.size();i++) {
			sd+= (laser_mins[i] - average_value) * (laser_mins[i] - average_value);
		}
		sd /=(double)laser_mins.size();
		sd = sqrt(sd);
		std::cout<<"AVERAGE MIN DISTANCE TO OBSTACLES: "<<average_value<<" meters. Standard Deviation = "<<sd<<std::endl;
	}
	

}

