#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(manette):
    
    speed = manette.axes[1]  # Utilisation de l'axe 1 pour la vitesse
    angular = manette.axes[3]  # Utilisation de l'axe 3 pour la direction

    cmd = Twist()
    cmd.linear.x = speed  # La vitesse linéaire est définie par l'axe 1
    cmd.angular.z = angular  # La vitesse angulaire est définie par l'axe 3
    pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('joystick_controller')  # Initialisation du nœud

    rospy.Subscriber('/joy', Joy, joy_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()  # Maintenir le programme actif
