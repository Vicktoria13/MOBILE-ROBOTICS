#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"
#include "iostream"

// on s'inscrit sur le topic /map.

// le call back doit entendre des messages de type OccupancyGrid
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  //cb de cases ?
  //ROS_INFO("j'entends : [%d]", msg->data[0]);

  ROS_INFO("taille map ligne = %d colonne = %d ", msg->info.height, msg->info.width); 

  cv::Mat opencv_matrix = cv::Mat::zeros( msg->info.height, msg->info.width, CV_8U);

  cv::Mat image = cv::imread("/home/spi-2019/Téléchargements/test.png");

  int cpt = 0;
  for (int i=0; i < 4000; i++){
    for (int j=0; j < 4000; j++){
        if (msg->data[i,j] == 0){
            cpt++;
        }
    }
  }
  ROS_INFO("tot = %d", cpt);

  // on veut ensuite afficher :
  
  cv::imshow("occupancy grid",image);
  cv::waitKey(10);
  

}
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "j_ecoute_la_map");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/map", 50, mapCallback);

  cv::namedWindow("occupancy grid",cv::WINDOW_NORMAL);
  ros::spin();
  cv::destroyWindow("occupancy grid");

  return 0;
}
