#include <ros/ros.h>
#include "path_planning_map/getUnicyclePos.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>



bool getUnicyclePosCallback(path_planning_map::getUnicyclePos::Request& req,
                             path_planning_map::getUnicyclePos::Response& res)
{
    // Lire la dernière position du robot depuis le topic /odom
    nav_msgs::Odometry::ConstPtr odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1.0));

    if (odom_msg)
    {
        res.unicycle_position = *odom_msg;
        return true;
    }
    else
    {
        ROS_ERROR("Aucun message reçu sur le topic /odom.");
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_position_server");
    ros::NodeHandle nh;

    // Créer un service pour récupérer la position initiale du robot
    ros::ServiceServer service = nh.advertiseService("/get_unicycle_pos", getUnicyclePosCallback);
    ROS_INFO("Pret pour donner la position du robot dans map ...");

    ros::spin();

    return 0;
}
