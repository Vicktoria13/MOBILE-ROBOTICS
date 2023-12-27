#!/usr/bin/env python
import getch
import rospy
from geometry_msgs.msg import Twist


################################
# created by yuvaram
#yuvaramsingh94@gmail.com
################################


def keys():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('clavier_teleop')
    rate = rospy.Rate(20)#try removing this line ans see what happens


    while not rospy.is_shutdown():
        k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        if ((k>=65)&(k<=68)|(k==115)|(k==113)|(k==97)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
            
            #si up, alors vitesse linéaire positive
            if k==65:
                speed=1
                angular=0
            #si down, alors vitesse linéaire négative
            elif k==66:
                speed=-1
                angular=0
            #si right, alors vitesse angulaire positive
            elif k==67:
                speed=0
                angular=1
            #si left, alors vitesse angulaire négative
            elif k==68:
                speed=0
                angular=-1

            #si s, alors vitesse nulle
            elif k==115:
                speed=0
                angular=0


                #publie la commande
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = angular
            pub.publish(cmd)

    
            


        

        #65 == up
        #66 == down
        #67 == right
        #68 == left
            
   
if __name__=='__main__':
    keys()