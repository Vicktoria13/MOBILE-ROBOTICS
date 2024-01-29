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
float x_goal_suiv;
float y_goal_suiv;
float x_curr;
float y_curr;
float theta_curr;
float v1;
float v2;
float err_x;
float err_y;
int indice_current_point = 0;
int nbrePoints = 0;
nav_msgs::Path chemin;

void topic_callback(const nav_msgs::Path& msg) {

  ROS_INFO("test");

  // Nombre de points dans la trajectoire
  nbrePoints = 0;
  indice_current_point = 0;
  for (geometry_msgs::PoseStamped p : msg.poses) nbrePoints++;

  ROS_INFO("Nombre de points : %d",nbrePoints);

  chemin = msg;

  // TODO : connaitre nombre de points dans la trajectoire
  // TODO : quand erreur faible, passer au point suivant


}

void topic_callback2(const nav_msgs::Odometry& msg2){
  
  //ROS_INFO("CURRENT X : %f",msg2.pose.pose.position.x);
  //ROS_INFO("CURRENT Y : %f",msg2.pose.pose.position.y);

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

  float theta_s,theta_e,x_RP,y_RP,d,k;

  while(ros::ok()){
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Lecture du point à atteindre
    ros::Subscriber sub1 = n.subscribe("trajectory", 1000, topic_callback);
    
    // Lecture position courante
    ros::Subscriber sub2 = n.subscribe("pose_unicycle", 1000, topic_callback2);

    // Distance l1 (centre du robot - point P)
    float l1 = 0.1;

    float u1, u2;
    
    // Calcul de l'erreur
    err_x = x_curr-x_goal;
    err_y = y_curr-y_goal;

    /*
    /// RALLIEMENT DE POINTS DE PASSAGE
    */

    ROS_INFO("NORME ERREUR : %f",err_x*err_x + err_y*err_y);

    ROS_INFO("X CURRENT : %lf",x_curr);
    ROS_INFO("Y CURRENT : %lf",y_curr);

    /*
    // Gains
    float k1 = 0.7;
    float k2 = 0.7;

    v1 = -k1*(x_curr-x_goal);
    v2 = -k2*(y_curr-y_goal);


    // Commande u1, u2 à envoyer au robot
    u1 = cos(theta_curr)*v1 + sin(theta_curr)*v2;
    u2 = (-sin(theta_curr)/l1)*v1 + (cos(theta_curr)/l1)*v2;

    //ROS_INFO("XG : %lf",x_goal);
    //ROS_INFO("YG : %lf",y_goal);

    //ROS_INFO("U1 : %lf",u1);
    //ROS_INFO("U2 : %lf",u2);
    */

    /*
    /// SUIVI DE CHEMIN
    */
    // Coordonnées du point courant à atteindre
    if (indice_current_point < nbrePoints-1){
      x_goal_suiv =  chemin.poses[indice_current_point+1].pose.position.x;
      y_goal_suiv =  chemin.poses[indice_current_point+1].pose.position.y;

      x_goal = chemin.poses[indice_current_point].pose.position.x;
      y_goal = chemin.poses[indice_current_point].pose.position.y;

      //ROS_INFO("XG_SUIV = %f",x_goal_suiv);
      //ROS_INFO("YG_SUIV = %f",y_goal_suiv);
      ROS_INFO("XG = %f",x_goal);
      ROS_INFO("YG = %f",y_goal);
      ROS_INFO("INDICE CURRENT POINT : %d", indice_current_point);
    }
    else{
      ROS_INFO("FIN");
    }

    // Calcul de theta_e avec les coordonnées du point actuel et du suivant
    theta_s = atan2((y_goal_suiv-y_goal),(x_goal_suiv-x_goal));
    theta_e = theta_curr-theta_s;

    //ROS_INFO("theta curr : %lf",theta_curr);
    //ROS_INFO("theta s : %lf",theta_s);
    ROS_INFO("theta e : %lf",theta_e);

    // Calcul des coordonnées du vecteur RP
    // R : origine du repère Rs
    x_RP = x_goal - (x_curr+l1*cos(theta_curr));
    y_RP = y_goal - (y_curr+l1*sin(theta_curr));

    // Calculer l'ordonnée de P dans Rs
    d = x_RP*sin(theta_s) + y_RP*cos(theta_s);

    //ROS_INFO("d : %lf",d);

    // Calcul du terme K
    if (1.57-abs(theta_e) > 0) k = 1;
    else k = 0;

    // Commandes
    if (indice_current_point < nbrePoints-1){
      u1 = 0.15;
      float u2_prov = -(u1*sin(theta_e))/(l1*abs(cos(theta_e))) - (u1/abs(cos(theta_e)))*k*d;
      float u2_sat = 0.8;

      // On utilise abs(cos(th_e)) car theta_e entre -pi et pi.

      //ROS_INFO("A : %lf",(u1*sin(theta_e)));
      //ROS_INFO("B : %lf",l1*cos(theta_e));
      //ROS_INFO("C : %lf",cos(theta_e));
      //ROS_INFO("u2_prov : %lf",u2_prov);

      if (abs(u2_prov) < u2_sat) u2 = u2_prov;
      else{
        if (u2_prov < 0) u2 = -u2_sat;
        else u2 = u2_sat;
      }
    }
    else{
      u1 = 0;
      u2 = 0;
    }

    //ROS_INFO("u1 : %lf",u1);
    //ROS_INFO("u2 : %lf",u2);


    // Envoi des commandes sur le topic cmd_vel
    cmd.linear.x = u1;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = u2;
    command.publish(cmd);

    // Si erreur faible, on passe au point suivant
    if (((err_x*err_x + err_y*err_y) < 0.6)) indice_current_point++;

    rate.sleep();
  }


  //ros::waitForShutdown();

  return 0;
}
