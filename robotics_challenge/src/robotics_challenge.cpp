#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
//JAC
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
// Costmap
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include "GlobalPlanner.h"


// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>

/* Marco*/
#define WORLD_SIZE 20 // tamaño del mapa en m
#define MAP_PRECISION 0.05 // precisión del mapa
#define MAP_SIZE (int) (WORLD_SIZE/MAP_PRECISION)  // tn de columnas y filas
#define ROI 0.5   // radio de influencia de los obstaculos
#define KFORCE 2  // para calibrar la fuerza repulsiva de los obstáculos
#define MIN_REPULSION_SIZE 0.4  // radio de la repulsión de los obstáculos
#define POINT_SEARCH_SIZE 0.5  // radio de busqueda del próximo punto
#define MIN_LOCAL_SIZE 0.2 // radio de detección du un mínimo local

 
double distance_map [MAP_SIZE][MAP_SIZE];    // mapa con pendiente hacia el objetivo
double repulsion_map [MAP_SIZE][MAP_SIZE]; // mapa con la repulsión de los obstáculos
std::vector<geometry_msgs::PoseStamped> obstacles;


tf::TransformListener *tf_ptr = NULL;
ros::Publisher *map_pose_pub_ptr = NULL;

struct MapPoint {
    int x;
    int y;
};

class Turtlebot {
public:
    Turtlebot();


    bool command(double goal_x, double goal_y);

    sensor_msgs::LaserScan data_scan;
    tf::TransformListener listener;
    tf::TransformListener xtion_listener;
    tf::TransformListener robot_listener;
    ros::Publisher marker_pub;


private:


    ros::NodeHandle nh_;

    //2D robot pose
    double x, y, theta;
    // Scan

    ros::Subscriber kinect_sub_;

    //Publisher and subscribers
    ros::Publisher vel_pub_;

    //!Publish the command to the turtlebot
    void publish(double angular_vel, double linear_vel);

    //!Callback for kinect
    void receiveKinect(const sensor_msgs::LaserScan & laser_kinect);

    double linear_vel;


};

Turtlebot::Turtlebot() {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    kinect_sub_ = nh_.subscribe("/scan", 1, &Turtlebot::receiveKinect, this);

    linear_vel = 0;

}

bool Turtlebot::command(double gx, double gy) {


    double angular_vel = 0.0;

    bool ret_val = false;

    //Transform the goal to the local frame
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped base_goal;

    goal.header.frame_id = "map";

    //we'll just use the most recent transform available for our simple example
    goal.header.stamp = ros::Time(0);

    //just an arbitrary point in space
    goal.pose.position.x = gx;
    goal.pose.position.y = gy;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    try {
        listener.transformPose("base_link", goal, base_goal);

        //  printf("goal: (%.2f, %.2f) -> base_goal: (%.2f, %.2f)\n", goal.point.x, goal.point.y, base_goal.point.x, base_goal.point.y);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"map\" to \"base_link\": %s", ex.what());
        return ret_val;
    }


    double base_angle = atan2(base_goal.pose.position.y, base_goal.pose.position.x); // angulo hacia el objetivo
    double distance = sqrt(pow(base_goal.pose.position.x, 2) + pow(base_goal.pose.position.y, 2)); // distancia hacia el objetivo


    angular_vel = 0.7 * base_angle;

    if (linear_vel < distance / 2) { // aceleración
        linear_vel += 0.01;

    } else {
        linear_vel -= 0.03;   // deceleración

    }

    linear_vel -= abs(2 * base_angle); // reduce la velocidad lineal si el ańgulo es grande


    if (linear_vel < 0) {
        linear_vel = 0;
    }


    if (abs(angular_vel) > 1) // limita la velocidad angular entre -1 y +1
    {
        angular_vel /= abs(angular_vel);
    }

    //  ROS_INFO("angle = %.2f, distance = %.2f", base_angle, distance);
    // linear_vel = 0;

    publish(angular_vel, linear_vel);
    return true;
}



//Publish the command to the turtlebot

void Turtlebot::publish(double angular, double linear) {
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    //std::cout << "Velocidades: " << vel.linear.x << ", " << vel.angular.z << std::endl;

    vel_pub_.publish(vel);


    return;
}

//Callback for robot position

void Turtlebot::receiveKinect(const sensor_msgs::LaserScan& msg) {
    data_scan = msg;
    // Different variables used to detect obstacles


}

void visualizePlan(geometry_msgs::PoseStamped goal, const std::vector<geometry_msgs::PoseStamped> &obstacles, ros::Publisher &marker_pub);
void setRepulsionZero();
std::vector<geometry_msgs::PoseStamped> loadPlan(const char *filename);
geometry_msgs::PoseStamped worldPosition(int x, int y);
void setAttraction(geometry_msgs::PoseStamped goal);
struct MapPoint mapPosition(double x, double y);
std::vector<geometry_msgs::PoseStamped> laserScan(sensor_msgs::LaserScan data_scan, tf::TransformListener & xtion_listener);
void setRepulsion(std::vector<geometry_msgs::PoseStamped> & new_obstacles);
double euclideanDistance(double x0, double y0, double x1, double y1);
geometry_msgs::PoseStamped getRobotPosition(tf::TransformListener & robot_listener);
struct MapPoint nextPoint(struct MapPoint robot_map_position);
struct MapPoint nextPoint(struct MapPoint robot_map_position);
void localMinimum(geometry_msgs::PoseStamped robot_position, struct MapPoint robot_map_position);
double getForce(struct MapPoint);
double getForce(int x, int y);

void publishPath(ros::Publisher path_pub, std::vector<geometry_msgs::PoseStamped> plan) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.type = 4;
    marker.id = 0;
    marker.action = 0;

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.scale.x = 0.05;
    for (unsigned i = 0; i < plan.size(); i++) {
        geometry_msgs::Point p;
        p.x = plan[i].pose.position.x;
        p.y = plan[i].pose.position.y;
        p.z = 0;
        marker.points.push_back(p);
    }
    path_pub.publish(marker);


}

void odomReceived(const nav_msgs::Odometry::ConstPtr& odom) {
    geometry_msgs::PoseStamped poseIn, poseOut;
    poseIn.pose = odom->pose.pose;
    poseIn.header = odom->header;
    poseIn.header.stamp = ros::Time(0);
    try {
        tf_ptr->transformPose("odom", poseIn, poseOut);
        poseOut.header.stamp = odom->header.stamp;
        map_pose_pub_ptr->publish(poseOut);
    } catch (std::exception &e) {
        ROS_ERROR("%s", e.what());
    }
}
/**
 * devuelve la fuerza repulsiva en el punto
 * @param point
 * @return 
 */
double getForce(struct MapPoint point) {
    return distance_map[point.x][point.y] + repulsion_map[point.x][point.y];
}
/**
 * devuelve la fuerza repulsiva en el punto
 * @param x
 * @param y
 * @return 
 */
double getForce(int x, int y) {
    return distance_map[x][y] + repulsion_map[x][y];
}

/**
 * aumenta la repulsión en el radio MIN_REPULSION_SIZE alrededor del robot.
 * @param robot_position
 * @param robot_map_position
 */
void localMinimum(geometry_msgs::PoseStamped robot_position, struct MapPoint robot_map_position) {
    for (int i = robot_map_position.x - (int) (MIN_REPULSION_SIZE / MAP_PRECISION); i < robot_map_position.x + (int) (MIN_REPULSION_SIZE / MAP_PRECISION); i++) {
        for (int j = robot_map_position.y - (int) (MIN_REPULSION_SIZE / MAP_PRECISION); j < robot_map_position.y + (int) (MIN_REPULSION_SIZE / MAP_PRECISION); j++) {

            if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) {
                geometry_msgs::PoseStamped other_position = worldPosition(i, j);
                double distance = euclideanDistance(robot_position.pose.position.x, robot_position.pose.position.y, other_position.pose.position.x, other_position.pose.position.y);
                if (distance < MIN_REPULSION_SIZE) {
                   
                    repulsion_map[i][j] += 0.5;


                }

            }
        }
    }
}

/**
 * devuelve el punto en el centro de la posición i j del mapa.
 * @param x
 * @param y
 * @return 
 */
geometry_msgs::PoseStamped worldPosition(int x, int y) {
    geometry_msgs::PoseStamped position;
    position.pose.position.x = (x - MAP_SIZE / 2) * MAP_PRECISION;
    position.pose.position.y = (y - MAP_SIZE / 2) * MAP_PRECISION;
    position.pose.position.z = 0;

    return position;
}

/**
 * Resetea la matriz de repulsión
 */
void setRepulsionZero() {
    for (uint i = 0; i < MAP_SIZE; i++) {
        for (uint j = 0; j < MAP_SIZE; j++) {
            repulsion_map[i][j] = 0;
        }
    }
}

/**
 * devuelve en un radio POINT_SEARCH_SIZE alrededor del robot el punto con menor repulsión
 * @param robot_map_position
 * @return 
 */
struct MapPoint nextPoint(struct MapPoint robot_map_position) {
    double force = getForce(robot_map_position);
    struct MapPoint next_point = robot_map_position;
    for (int i = robot_map_position.x - (int) (POINT_SEARCH_SIZE / MAP_PRECISION); i < robot_map_position.x + (int) (POINT_SEARCH_SIZE / MAP_PRECISION); i++) {
        for (int j = robot_map_position.y - (int) (POINT_SEARCH_SIZE / MAP_PRECISION); j < robot_map_position.y + (int) (POINT_SEARCH_SIZE / MAP_PRECISION); j++) {
            if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) {
                double local_force = distance_map[i][j] + repulsion_map[i][j];
                if (local_force < force) {
                    force = local_force;
                    next_point.x = i;
                    next_point.y = j;
                }

            }
        }
    }
    return next_point;
}

/**
 * devuelve la posición del robot 
 * @param robot_listener
 * @return 
 */
geometry_msgs::PoseStamped getRobotPosition(tf::TransformListener & robot_listener) {

    geometry_msgs::PoseStamped robot_base;

    geometry_msgs::PoseStamped odom_base; // posición de la base del robot

    robot_base.header.frame_id = "base_link";

    robot_base.header.stamp = ros::Time(0);


    robot_base.pose.position.x = 0.0;
    robot_base.pose.position.y = 0.0;
    robot_base.pose.position.z = 0.0;
    robot_base.pose.orientation.x = 0;
    robot_base.pose.orientation.y = 0;
    robot_base.pose.orientation.z = 0;
    robot_base.pose.orientation.w = 1;
    while (true) {


        try {
            robot_listener.transformPose("map", robot_base, odom_base); // transformo la posición del robot 
            break;
        } catch (tf::TransformException& ex) {
                   ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"odom\": %s", ex.what());

        }
    }

    return odom_base;

}

/**
 * recibe un vector de obstáculos y crea fuerza de repulsión alrededor de cada uno de ellos, en un radio ROI
 * @param new_obstacles
 */
void setRepulsion(std::vector<geometry_msgs::PoseStamped>& new_obstacles) {
    for (uint n = 0; n < new_obstacles.size(); n++) { // para cada nuevo obstáculo
        geometry_msgs::PoseStamped obstacle = new_obstacles[n];
        struct MapPoint map_obstacle = mapPosition(obstacle.pose.position.x, obstacle.pose.position.y); // posición en el mapa del obstáculo
        repulsion_map[map_obstacle.x][ map_obstacle.y] = DBL_MAX - distance_map[map_obstacle.x][ map_obstacle.y];

        for (uint i = map_obstacle.x - (int) (ROI / MAP_PRECISION); i < map_obstacle.x + (int) (ROI / MAP_PRECISION); i++) { // desde 1m menos del obstaculo en el eje x
            for (uint j = map_obstacle.y - (int) (ROI / MAP_PRECISION); j < map_obstacle.y + (int) (ROI / MAP_PRECISION); j++) { // desde 1m menos del obstaculo en el eje y
                if (i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE) { // compruebo que no se salga del rango del mapa

                    geometry_msgs::PoseStamped my_point = worldPosition(i, j);

                    double dist = euclideanDistance(my_point.pose.position.x, my_point.pose.position.y, obstacle.pose.position.x, obstacle.pose.position.y); // distancia del obstaculo al punto en el mapa

                    if (dist < ROI && i != map_obstacle.x && j != map_obstacle.y) { // si distancia es < 1 
                        double force = ((ROI - dist) / dist - 0.2) * KFORCE; // cambiar el coeficiente para calibrar la fuerza.
                        if (!isnan(force)) {
                            if (repulsion_map[i][j] < force) {// si la fuerza en el punto es menor, la substituyo.
                            repulsion_map[i][j] = force;
                        }
                        } 
                    }

                }
            }

        }

    }
}

/**
 * devuelve en un vector las lecturas de los láseres
 * @param data_scan
 * @param xtion_listener
 * @return 
 */
std::vector<geometry_msgs::PoseStamped> laserScan(sensor_msgs::LaserScan data_scan, tf::TransformListener & xtion_listener) {
    std::vector<geometry_msgs::PoseStamped> rob_obstacles;
    std::vector<geometry_msgs::PoseStamped> odom_obstacles;


    /*Atributos del escaner laser*/
    double angle_increment = data_scan.angle_increment;
    double angle_min = data_scan.angle_min;
    int nLasers = data_scan.ranges.size(); // 640 lasers
    double angle = angle_min;


    /*Lectura de los lasers*/
    for (uint i = 0; i < nLasers; i++, angle += angle_increment) {


        double dist = data_scan.ranges[i];
        if (!isnan(dist)) {
            geometry_msgs::PoseStamped obs;
            obs.pose.position.x = dist * cos(angle);
            obs.pose.position.y = dist * sin(angle);
            obs.pose.position.z = 0.0;

            rob_obstacles.push_back(obs); // se guardan los obstaculos detectados
        }
    }

    for (uint i = 0; i < rob_obstacles.size(); i++) {

        geometry_msgs::PoseStamped rob_obs = rob_obstacles[i];
        geometry_msgs::PoseStamped odom_obs;
        rob_obs.pose.orientation.x = 0;
        rob_obs.pose.orientation.y = 0;
        rob_obs.pose.orientation.z = 0;
        rob_obs.pose.orientation.w = 1;
      
        rob_obs.header.frame_id = "mount_asus_xtion_pro_link";
        rob_obs.header.stamp = ros::Time(0);
     
        try {
            //TODO  xtion_listener.waitForTransform
            xtion_listener.transformPose("map", rob_obs, odom_obs);
        } catch (tf::TransformException& ex) {
           ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());

        }
       
        odom_obstacles.push_back(odom_obs);
    }
    obstacles.insert(obstacles.end(), odom_obstacles.begin(), odom_obstacles.end());
    return odom_obstacles;
}



/**
 * Para cada punto del mapa, busca la distancia mínima con los puntos del recorrido del A* (plan). 
 * Suma al punto del mapa la distancia mínima hacia plan y resta el índice del punto más próximo, para crear una pendiente hacia el objetivo final
 * @param plan
 */
void setValley(std::vector<geometry_msgs::PoseStamped> plan) {

    geometry_msgs::PoseStamped tempGoal;
    for (uint i = 0; i < MAP_SIZE; i++) {
        for (uint j = 0; j < MAP_SIZE; j++) {
            geometry_msgs::PoseStamped point;
            point = worldPosition(i, j);
            double min = 9999999.0;
            int min_n;

            for (uint n = 0; n < plan.size(); n++) {
                double dist = euclideanDistance(plan[n].pose.position.x, plan[n].pose.position.y, point.pose.position.x, point.pose.position.y);
                if (dist < min) {
                    min = dist;
                    min_n = n;
                }
            }
            distance_map[i][j] = min - min_n * 0.1;

        }
    }

}

/**
 * devuelve la posición en el mapa de un punto
 * @param x
 * @param y
 * @return 
 */
struct MapPoint mapPosition(double x, double y) {
    struct MapPoint map_point;
    map_point.x = (int) (x / MAP_PRECISION) + MAP_SIZE / 2;
    map_point.y = (int) (y / MAP_PRECISION) + MAP_SIZE / 2;
    return map_point;

}

/**
 * devuelve la distancia euclidea entre dos puntos
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @return 
 */
double euclideanDistance(double x0, double y0, double x1, double y1) {
    return sqrt(pow(x1 - x0, 2) + pow(y0 - y1, 2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotics_challenge");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double start_x, start_y;
    double goal_x, goal_y;

    tf::TransformListener tf(ros::Duration(10));
    tf_ptr = &tf;

    costmap_2d::Costmap2DROS cost_map("cost_map", tf);

    // (start_x, start_y) are the coordinates of the initial position of the robot in MAP frame
    pn.param<double>("start_x", start_x, 3.46);
    pn.param<double>("start_y", start_y, 4.62);

    // (goal_x, goal_y) are the coordinates of the goal in MAP frame
    pn.param<double>("goal_x", goal_x, 6.0);
    pn.param<double>("goal_y", goal_y, 9.0);

    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
    ros::Publisher map_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/map_pose", 1);
    map_pose_pub_ptr = &map_pose_pub;

    // Start position
    geometry_msgs::PoseStamped start;
    start.pose.position.x = start_x;
    start.pose.position.y = start_y;
    start.pose.position.z = 0.0;
    start.header.stamp = ros::Time::now();

    // Goal Position
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = 0.0;
    goal.header.stamp = ros::Time::now();


    std::vector<geometry_msgs::PoseStamped> plan;

    global_planner::GlobalPlanner planner("turtle_planner", &cost_map);
    planner.makePlan(start, goal, plan);
    std::cout << "Plan size:  " << plan.size() << std::endl;
    Turtlebot robot;

    ros::Rate loop_rate(15);

    setRepulsionZero();
    setValley(plan);
  
    while (ros::ok()) {
        ros::spinOnce();
        publishPath(path_pub, plan);
        std::vector<geometry_msgs::PoseStamped> new_obstacles = laserScan(robot.data_scan, robot.xtion_listener);
        setRepulsion(new_obstacles);
        geometry_msgs::PoseStamped robot_position;
        struct MapPoint robot_map_position;
        struct MapPoint next_map_point;
        geometry_msgs::PoseStamped next_point;
        bool isMin = false;
        do {
            robot_position = getRobotPosition(robot.robot_listener);
            if(euclideanDistance(robot_position.pose.position.x,robot_position.pose.position.y, goal_x, goal_y )<0.2){
                                std::cout << "\n LLEGADOOOO!!!!!!!!!!!\n";
                                return 0;
            }
            robot_map_position = mapPosition(robot_position.pose.position.x, robot_position.pose.position.y);
            next_map_point = nextPoint(robot_map_position);
            next_point = worldPosition(next_map_point.x, next_map_point.y);
            robot.command(next_point.pose.position.x, next_point.pose.position.y);
            
         /*si el próximo punto encontrado se encuentra en un radio inferior a MIN_LOCAL_SIZE se llama la función localMinimum para crear un repulsión en el lugar.
          */
            if (MIN_LOCAL_SIZE > euclideanDistance(robot_position.pose.position.x, robot_position.pose.position.y, next_point.pose.position.x, next_point.pose.position.y)) {

                localMinimum(robot_position, robot_map_position);
                isMin = true;
                std::cout << "\n MINIMO!!!!!!!!!!!\n";

            } else {
                isMin = false;
            }
        } while (isMin);
  
        loop_rate.sleep();
    }
    return 0;
}