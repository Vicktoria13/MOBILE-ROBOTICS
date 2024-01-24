#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

float x_goal;
float y_goal;
float x_curr;
float y_curr;
float theta_curr;
float err_x;
float err_y;
float obs_x;
float obs_y;
float obs_angle;
float relative_angle;

float angle_obstacle;
float dist_obstacle;
float dist_ahead;
float dist_left_eye;
float dist_left;
float dist_right_eye;
float dist_right;


void topic_callback(const nav_msgs::Path& msg) {

  x_goal = msg.poses[0].pose.position.x;
  y_goal = msg.poses[0].pose.position.y;

}

void topic_callback2(const nav_msgs::Odometry& msg2){
  // ROS_INFO("CURRENT X : %f",msg2.pose.pose.position.x);
  // ROS_INFO("CURRENT Y : %f",msg2.pose.pose.position.y);

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

  // ROS_INFO("CURRENT THETA : %f",theta_curr);
  // ROS_INFO ("rot x : %f",rot_x);
  // ROS_INFO ("rot y : %f",rot_y);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg3){

  // Extraire l'information du message du LaserScan
  float angle_min = msg3->angle_min;
  float angle_increment = msg3->angle_increment;
  float angle_max = msg3->angle_max;
  float range_min = msg3->range_min;
  float range_max = msg3->range_max;
  std::vector<float> ranges = msg3->ranges;

  angle_obstacle = -3;
  dist_ahead = range_max;
  dist_left_eye = range_max;
  dist_right_eye = range_max;

  // Récupération de l'obstacle le plus lointain
  //for (size_t i = 0; i < ranges.size(); ++i){
    //if (ranges[i] >= range_min && ranges[i] <= range_max){

  //obs_angle = angle_min + i*angle_increment;
  //obs_x = ranges[i] * std::cos(obs_angle);
  //obs_y = ranges[i] * std::sin(obs_angle);



  // On récupère la distance en face et les distances -45° et +45°
  // On tourne le robot à -45°, +45°, -90° ou + 90°
  dist_ahead = ranges[360];
  dist_right_eye = ranges[220];
  dist_left_eye = ranges[500];
  dist_left = ranges[640];
  dist_right = ranges[80];

  dist_obstacle = dist_right_eye;
  angle_obstacle = -0.4;
  if (dist_left_eye > dist_obstacle){
    dist_obstacle = dist_left_eye;
    angle_obstacle = 0.4;
  }
  if (dist_left > dist_obstacle){
    dist_obstacle = dist_left;
    angle_obstacle = 0.8;
  }
  if (dist_right > dist_obstacle){
    dist_obstacle = dist_right;
    angle_obstacle = -0.8;
  }

   ///}
  //}

  relative_angle = angle_obstacle - theta_curr;

  if (relative_angle < 0){
    //ROS_INFO("Obstacle à droite : Angle = %f", angle_obstacle);
  } else {
    //ROS_INFO("Obstacle à gauche : Angle = %f", angle_obstacle);
  }

  //ROS_INFO("distance avec obstacle : %lf",dist_obstacle);
  //ROS_INFO("distance devant : %lf",dist_ahead);
  //ROS_INFO("distance gauche : %lf",dist_left_eye);
  //ROS_INFO("distance droite : %lf",dist_right_eye);

    // ROS_INFO("Obstacle at (x,y) = (%f, %f)", obs_x, obs_y);
 
    //ROS_INFO("i : %lf",ranges[i]);
    //ROS_INFO("increm : %lf",angle_increment); // 0.0056 rad = 0.3°
    //ROS_INFO("%d",ranges.size());             // 720
    //ROS_INFO("min : %lf",angle_min);          // -2 rad = -115°
    //ROS_INFO("max : %lf",angle_max);          //  2 rad = 115°
    //ROS_INFO("range min : %lf",range_min);    // 0.1 m
    //ROS_INFO("range max : %lf",range_max);    // 15 m
}

int main(int argc, char **argv){
  ros::init(argc,argv,"cmd_auto");
  ros::NodeHandle n;

  ros::Rate rate(1);  // Fréquence de publication (1 Hz, par exemple)

  ros::Publisher command = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist cmd;

  while(ros::ok()){
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Lecture du point à atteindre
    //ros::Subscriber sub1 = n.subscribe("trajectory", 1000, topic_callback);
    
    // Lecture position courante
    //ros::Subscriber sub2 = n.subscribe("odom", 1000, topic_callback2);

    ros::Subscriber sub3 = n.subscribe("scan", 1000, scan_callback);
    
    // Détection d'obstacle
    // Si on rencontre un obstacle, on tourne de 45°
    // sinon on continue tout droit

    bool obstacle_detected = false;

    // Vitesse désirée 
    float u_des = 0.0; // 2 cm/s
    
    // Commande u, omega à envoyer au robot
    float u, omega;
    u = u_des;

    //ROS_INFO("Obstacle at (x,y) = (%f, %f)", obs_x, obs_y);

    // Si pas de mur en face, tout droit
    // Sinon, on se tourne vers l'obstacle le plus lointain

    if (dist_ahead < 0.6 || dist_left_eye < 0.6 || dist_right_eye < 0.6) {

      ROS_INFO("Obstacle detected");
      obstacle_detected = true;

      omega = angle_obstacle;
      u = 0.0;

      /*
      if ((relative_angle > 0) && (relative_angle > 1)){
        ROS_INFO("Obstacle to the right : Angle = %f", relative_angle);
        omega = 0.7845; // tourner à gauche
      } else if ((relative_angle < 0) && (relative_angle < -1)){
        ROS_INFO("Obstacle to the left : Angle = %f", relative_angle);
        omega = - 0.7845; // tourner à droite
      }*/

    } else {
      obstacle_detected = false;

      omega = 0.0; 
      u = 0.3;
    }

    // omega = (k1*(d-d0))/(u*cos(theta_curr-gamma)) - k2*tan(theta_curr-gamma);
    // à définir: d, d0, et gamma

    ROS_INFO("U : %lf",u);
    ROS_INFO("omega : %lf",omega);

    cmd.linear.x = u;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = omega;
    command.publish(cmd);

    rate.sleep();


  }


  //ros::waitForShutdown();

  return 0;
}