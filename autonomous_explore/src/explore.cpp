#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

class CommandPublisher
{
public:
    CommandPublisher(const std::string& savePath) : save_path(savePath)
    {
        ros::NodeHandle nh;

        map_subscriber = nh.subscribe("map", 1, &CommandPublisher::mapCallback, this);
        odom_subscriber = nh.subscribe("odom", 1, &CommandPublisher::odomCallback, this);
        cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
      


        
    }

    // Callback pour le topic "odom"
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

        cmd_vel_publisher.publish(cmd_vel_msg);
    }


    


   

private:
    ros::Subscriber map_subscriber;
    ros::Subscriber odom_subscriber;
    ros::Publisher cmd_vel_publisher;
    std::string save_path;

    float pos_x_metres;
    float pos_y_metres;
    float pos_theta_radians;

};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Veuillez fournir le chemin de sauvegarde en argument.");
        return 1;
    }

    std::string savePath(argv[1]);

    // Initialiser le nœud ROS
    ros::init(argc, argv, "command_publisher_node");

    CommandPublisher commandPublisher(savePath);

    ros::spin();

    return 0;
}