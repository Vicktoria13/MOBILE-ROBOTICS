#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"
#include <cmath>
#include <tf/tf.h>

float x_goal;
float y_goal;
float x_curr;
float y_curr;
float theta_curr;
float v1;
float v2;
float err_x;
float err_y;
int indice_current_point = 0;

void topic_callback(const nav_msgs::Path& msg) {

  // Nombre de points dans la trajectoire
  int nbrePoints = 0;
  for (geometry_msgs::PoseStamped p : msg.poses) nbrePoints++;

  // Coordonnées du point courant à atteindre
  //if (indice_current_point < nbrePoints){
    x_goal = msg.poses[indice_current_point].pose.position.x;
    y_goal = msg.poses[indice_current_point].pose.position.y;
  //}

  ROS_INFO("INDICE CURRENT POINT : %d", indice_current_point);

  // TODO : connaitre nombre de points dans la trajectoire
  // TODO : quand erreur faible, passer au point suivant


}

void topic_callback2(const nav_msgs::Odometry& msg2){
  
  ROS_INFO("CURRENT X : %f",msg2.pose.pose.position.x);
  ROS_INFO("CURRENT Y : %f",msg2.pose.pose.position.y);

  x_curr = msg2.pose.pose.position.x;
  y_curr = msg2.pose.pose.position.y;

  // Conversion des quaternions pour récupérer la rotation autour de z
  float qx = msg2.pose.pose.orientation.x;
  float qy = msg2.pose.pose.orientation.y;
  float qz = msg2.pose.pose.orientation.z;
  float qw = msg2.pose.pose.orientation.w;

  tf::Quaternion quat(qx,qy,qz,qw);
  tf::Matrix3x3 mat(quat);

  double rot_x,rot_y,rot_z;
  mat.getEulerYPR(rot_z,rot_y,rot_x);     // Récupération des rotations en radians

  theta_curr = rot_z;

  //ROS_INFO("CURRENT THETA : %f",theta_curr);
  //ROS_INFO ("rot x : %f",rot_x);
  //ROS_INFO ("rot y : %f",rot_y);
}


int main(int argc, char **argv){
  ros::init(argc,argv,"read_traj");
  ros::NodeHandle n;

  ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

  ros::Publisher command = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist cmd;

  while(ros::ok()){
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Subscriber sub1 = n.subscribe("trajectory", 1000, topic_callback);
    ros::Subscriber sub2 = n.subscribe("odom", 1000, topic_callback2);
    
    // Calcul de l'erreur
    err_x = x_curr-x_goal;
    err_y = y_curr-y_goal;

    ROS_INFO("ERREUR X : %f",err_x);
    ROS_INFO("ERREUR Y : %f",err_y);

    // Envoi de la commande
    float k1 = 0.2;
    float k2 = 0.8;

    v1 = -k1*(x_curr-x_goal);
    v2 = -k2*(y_curr-y_goal);

    // Distance l1 (centre du robot - point P)
    float l1 = 0.1;

    // Commande u1, u2 à envoyer au robot
    float u1, u2;
    u1 = cos(theta_curr)*v1 + sin(theta_curr)*v2;
    u2 = (-sin(theta_curr)/l1)*v1 + (cos(theta_curr)/l1)*v2;

    ROS_INFO("XG : %lf",x_goal);
    ROS_INFO("YG : %lf",y_goal);

    //ROS_INFO("U1 : %lf",u1);
    //ROS_INFO("U2 : %lf",u2);

    cmd.linear.x = u1;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = u2;
    command.publish(cmd);

    // Si erreur faible, on passe au point suivant
    if ((err_x*err_x + err_y*err_y) < 0.1) indice_current_point++;

    rate.sleep();
  }


  //ros::waitForShutdown();

  return 0;
}