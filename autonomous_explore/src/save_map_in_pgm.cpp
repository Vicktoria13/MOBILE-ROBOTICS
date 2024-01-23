

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "path_planning_map/Divide.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>




/**
 * @brief Noeud recuperer de https://github.com/ros-planning/navigation/blob/melodic-devel/map_server/src/map_saver.cpp
 * qui permet de lancer le map saver en permanence
 * 
 */
using namespace std;

class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free)
      : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      odom_subscriber = n.subscribe("odom", 1, &MapGenerator::odomCallback, this);
      cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    }



    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
     
        //on enregistre la position du robot
        pos_x_metres = odom_msg->pose.pose.position.x;
        pos_y_metres = odom_msg->pose.pose.position.y;
        pos_theta_radians = tf::getYaw(odom_msg->pose.pose.orientation);

        publishCommand();
    }

    // Fonction pour publier une commande de vitesse
    void publishCommand()
    {
        geometry_msgs::Twist cmd_vel_msg;

        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.1;

        //cmd_vel_publisher.publish(cmd_vel_msg);
    }



    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
   
      image_map = cv::Mat::zeros(map->info.height, map->info.width, CV_8UC1);

      ROS_INFO("map received");



      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            image_map.at<uchar>(y,x) = 255;
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            image_map.at<uchar>(y,x) = 0;

          } else { //occ [0.25,0.65]
            image_map.at<uchar>(y,x) = 0;
          }
        }
      }


      this->resolution = map->info.resolution;
      this->origin[0] = map->info.origin.position.x;
      this->origin[1] = map->info.origin.position.y;
      this->origin[2] = 0.0;

      build_grid_from_pgm(pos_x_metres, pos_y_metres, pos_theta_radians);
      ROS_INFO("Done\n");
    }


    void build_grid_from_pgm(float pos_x_in_map, float pos_y_in_map, float orientation_in_map){

      Divide div = Divide(image_map, 3);
      div.divide_map_for_exploring();

      // conversion
      int x_subcells;
      int y_subcells;
      ROS_INFO("resolution = %f",resolution);
      int a = div.convert_from_meters_to_free_subcells(pos_x_in_map,pos_y_in_map,&x_subcells,&y_subcells,resolution,origin,false,true);
      ROS_INFO("x_subcells = %d, y_subcells = %d",x_subcells,y_subcells);
      div.display_exploration("/home/victoria/robmob_ws/src/autonomous_explore/map/", x_subcells, y_subcells);
    
    }




    //attribut


    std::string mapname_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_subscriber;
    ros::Publisher cmd_vel_publisher;

    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;
    cv::Mat image_map;

    float pos_x_metres;
    float pos_y_metres;
    float pos_theta_radians;

    double resolution;
    double* origin = new double[3];



};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "/home/victoria/robmob_ws/src/autonomous_explore/map/exploring_map";

  int threshold_occupied = 65;
  int threshold_free = 19;


  MapGenerator mg(mapname, threshold_occupied, threshold_free);

  while(!mg.saved_map_ && ros::ok())
    ros::spin();

  return 0;
}
