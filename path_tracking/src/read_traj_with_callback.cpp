#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf2_msgs/TFMessage.h"
#include <cmath>
#include <tf/tf.h>


class FollowPathNode {


private:
    ros::NodeHandle nh_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    ros::Rate rate_ = ros::Rate(1);  

    float x_goal_;
    float y_goal_;

    float x_curr_;
    float y_curr_;
    float theta_curr_;

    float v1_;
    float v2_;

    float err_x_;
    float err_y_;


    float k1;
    float k2;

    // ÎNDICE DU POINT COURANT DE la TRAJECTOIRE


public:

    FollowPathNode(ros::NodeHandle& nh) : nh_(nh) {


        trajectory_sub_ = nh_.subscribe("/trajectory", 1, &FollowPathNode::trajectoryCallback, this);
        odom_sub_ = nh_.subscribe("/pos_unicycle", 1, &FollowPathNode::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // GAINS
        k1=0.05;
        k2=0.02;

    }

    void run() {
        while (ros::ok()) {
            ros::spinOnce();
            rate_.sleep();
        }
    }



    void trajectoryCallback(const nav_msgs::Path& msg) {
        /**
         * @brief S'execute dès qu'un message est publié sur le topic "trajectory"
         * 
         */

        // on fait la commande ici
        // car si on fait la commande dans odomCallback, alors la commande n'arrive pas si le robot est immobile

        ROS_INFO("Nouvelle trajectoire reçue == > Calcul et envoie de la commande");
        ROS_INFO("il y a %d points dans la trajectoire", msg.poses.size());

        int indice_current_point_ = 0;

        while( indice_current_point_ < msg.poses.size() ) {

            // Récupération du point courant
            x_goal_ = msg.poses[indice_current_point_].pose.position.x;
            y_goal_ = msg.poses[indice_current_point_].pose.position.y;


            // Calcul de l'erreur
            err_x_ = x_curr_ - x_goal_;
            err_y_ = y_curr_ - y_goal_;

            // Calcul de la commande
            v1_ = -k1 * err_x_;
            v2_ = -k2 * err_y_;

            float l1 = 0.1;  // Distance l1 (centre du robot - point P)

            // Commande u1, u2 à envoyer au robot
            float u1, u2;
            u1 = cos(theta_curr_) * v1_ + sin(theta_curr_) * v2_;
            u2 = (-sin(theta_curr_) / l1) * v1_ + (cos(theta_curr_) / l1) * v2_;

            ROS_INFO("u1 = %f m/s", u1);
            ROS_INFO("u2 = %f rad/s", u2);




            // Publication de la commande
            geometry_msgs::Twist cmd;
            cmd.linear.x = u1;
            cmd.angular.z = u2;

            cmd_pub_.publish(cmd);

            // Si l'erreur est faible, on passe au point suivant
            if (sqrt(err_x_*err_x_ + err_y_*err_y_) < 0.1) {
                indice_current_point_++;
            }

            rate_.sleep();
        }
    
    }

    void odomCallback(const nav_msgs::Odometry& msg) {
        /**
         * @brief S'execute dès qu'un message est publié sur le topic "pos_unicycle"
         * S'occupe juste de mettre à jour les variables x_curr_, y_curr_ et theta_curr_
         */

        x_curr_ = msg.pose.pose.position.x;
        y_curr_ = msg.pose.pose.position.y;

        // Conversion des quaternions pour récupérer la rotation autour de z
        float qx = msg.pose.pose.orientation.x;
        float qy = msg.pose.pose.orientation.y;
        float qz = msg.pose.pose.orientation.z;
        float qw = msg.pose.pose.orientation.w;

        tf::Quaternion quat(qx, qy, qz, qw);
        tf::Matrix3x3 mat(quat);

        double rot_x, rot_y, rot_z;
        mat.getEulerYPR(rot_z, rot_y, rot_x); // Récupération des rotations en radians

        theta_curr_ = rot_z;


        
    }


       
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "commande");
    ros::NodeHandle nh("~");

    FollowPathNode followerNode(nh);
    followerNode.run();

    return 0;
}

