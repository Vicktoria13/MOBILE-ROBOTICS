#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "path_planning_map/Divide.hpp"



class MyNode {
public:
    MyNode() {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        map_sub_ = nh_.subscribe("/map", 1, &MyNode::mapCallback, this);

        // Set the service client to call the dynamic map service
        dynamic_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/dynamic_map");

        // Set the update rate (e.g., every 5 seconds)
        update_rate_ = 0.8; // Set the desired update rate in Hz
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MyNode::timerCallback, this);
    }

    // Callback for the /map topic
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        
        // ROS info la taille de la map
        ROS_INFO("Map size: %d x %d", map_msg->info.width, map_msg->info.height);
        
    }



    cv::Mat occupancy_to_cvmatrix(const nav_msgs::OccupancyGrid& map_msg) {
    // Convert the map message to a cv::Mat

        // Dimensions limites
        int N = 5;
        int W_MIN = ((N-1)/2)*map_msg.info.width/N;
        int W_MAX = ((N+1)/2)*map_msg.info.width/N;
        int H_MIN = ((N-1)/2)*map_msg.info.height/N;
        int H_MAX = ((N+1)/2)*map_msg.info.height/N;

        ROS_INFO("New map size: %d x %d", (W_MAX-W_MIN), (H_MAX-H_MIN));

        //pixel dimensions souhaitées EN RIS !
        // 0 est la couleur noire, 255 la couleur blanche
        cv::Mat map_image((H_MAX-H_MIN), (W_MAX-W_MIN), CV_8UC1);
        int seuil = 50;

        for (int r = W_MIN; r < W_MAX; r++) {
            for (int c = H_MIN; c < H_MAX; c++) {
                int8_t map_value = map_msg.data[r * map_msg.info.width + c];

                //map_value est une probabilité entre 0 et 100, -1 si inconnu
                if (map_value == -1) {

                    map_image.at<uchar>(r-W_MIN, c-H_MIN) = 0; //obstacle = inconnu

                } else if (map_value > seuil) {

                    map_image.at<uchar>(r-W_MIN, c-H_MIN) = 0;

                } else {
                    map_image.at<uchar>(r-W_MIN, c-H_MIN) = 255;
                }

            }
        }


    return map_image;
    }





    // Timer callback for periodic map updates
    void timerCallback(const ros::TimerEvent& event) {
        // Call the dynamic map service to get the updated map
        ROS_INFO("Timer callback");
        nav_msgs::GetMap dynamic_map_srv;
        if (dynamic_map_client_.call(dynamic_map_srv)) {
            ROS_INFO("Received updated map from dynamic_map service");

            //on affiche charge une image png
            cv::Mat map_image = occupancy_to_cvmatrix(dynamic_map_srv.response.map);
            cv::imshow("Map", map_image);
            cv::waitKey(30);

            //save in grayscale
            cv::imwrite("/home/spi-2019/test1.png", map_image);

            // TRAITEMENT DE LA MAP
            Divide div = Divide(map_image, 10);
            //print dimensions
            ROS_INFO("rows: %d", div.get_rows());

        } else {
            ROS_ERROR("Failed to call dynamic_map service");
        }


        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::ServiceClient dynamic_map_client_;
    ros::Timer timer_;
    double update_rate_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    MyNode my_node;
    ros::spin();

    return 0;
}