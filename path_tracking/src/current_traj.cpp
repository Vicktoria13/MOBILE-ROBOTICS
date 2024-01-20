#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener_node");
    ros::NodeHandle nh;

    // Créer un TransformListener
    tf::TransformListener tf;
    tf::StampedTransform transform;

    nav_msgs::Path traj_msg;
    traj_msg.header.frame_id = "map";
    traj_msg.header.stamp = ros::Time::now();


    //permet de publier la pose actuelle du robot
    ros::Publisher pose_pub = nh.advertise<nav_msgs::Odometry>("/pose_unicycle", 1);
    ros::Publisher traj_pub = nh.advertise<nav_msgs::Path>("/current_trajectory", 1);



    ros::Rate rate(10.0);

    while (ros::ok()) {

        try {
            // Essayer de trouver la transformation entre /map et /base_link
            //wait for transform to be available
            tf.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(3.0));
            tf.lookupTransform("odom", "base_footprint", ros::Time(0), transform);

        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN("No transform between /map and /base_link");
            ros::Duration(3.0).sleep();
            continue;
        }



        // Créer un message de type Odometry
        nav_msgs::Odometry pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();

        // Remplir le message
        pose_msg.pose.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.pose.position.z = transform.getOrigin().z();

        pose_msg.pose.pose.orientation.x = transform.getRotation().x();
        pose_msg.pose.pose.orientation.y = transform.getRotation().y();
        pose_msg.pose.pose.orientation.z = transform.getRotation().z();
        pose_msg.pose.pose.orientation.w = transform.getRotation().w();

        // Publier le message
        pose_pub.publish(pose_msg);

        // Créer un message de type Path
        
        // Remplir le message
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = transform.getOrigin().x();
        pose.pose.position.y = transform.getOrigin().y();
        pose.pose.position.z = transform.getOrigin().z();

        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();

        traj_msg.poses.push_back(pose);
        

        ROS_INFO("il y a %d poses dans le message", traj_msg.poses.size());

        // Publier le message
        traj_pub.publish(traj_msg);

      

        // Attendre selon la fréquence définie
        rate.sleep();

        
    }

    return 0;
}
