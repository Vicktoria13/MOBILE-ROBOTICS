#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"


void topic_callback(const geometry_msgs::PoseArray& msg) {

  // Récupérer le nombre de points dans la trajectoire
  int nbrePoints = sizeof(msg.poses)/4;

  for (int i = 0; i < nbrePoints; i++){
    ROS_INFO("ID: %d",i+1);
    ROS_INFO("POS X:%lf",msg.poses[i].position.x);
    ROS_INFO("POS Y:%lf",msg.poses[i].position.y);
    ROS_INFO("\n");
  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "read_traj");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("trajectory", 1000, topic_callback);
  ros::spin();

  return 0;
}