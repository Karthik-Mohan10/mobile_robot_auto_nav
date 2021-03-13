#!/usr/bin/env python

#This code is for manual maneuver of mobile robot using keystrokes

import rospy
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import pygame

def velcommand(v,w):
    return Twist(Vector3(v,0,0),Vector3(0,0,w))


#Function to assign speed (linear and angular) to the mobile robot
def speed():

    pygame.init()
    pygame.display.set_mode((100,100))
    pressed_up  = False
    pressed_down  = False
    pressed_left  = False
    pressed_right  = False


    velcommand(0,0)

    #Velocity parameters 
    V_MAX = 0.1
    W_MAX = 0.2
    v=0
    w=0
    a=2
    al=2
    dt=0.01
    dtw=0.01
    reduc_v=0.1
    reduc_w=0.1
    thresh_v=0.001
    thresh_w=0.001


    #ROS node intialisation 
    rospy.init_node("keyboard_controller",anonymous=True)
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)

    
    #Velocity assigned based on the input from keystrokes
    while not rospy.is_shutdown():
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    pressed_up = True
                if event.key == pygame.K_DOWN:
                    pressed_down = True
                if event.key == pygame.K_LEFT:
                    pressed_left = True
                if event.key == pygame.K_RIGHT:
                    pressed_right = True

            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_UP:
                    pressed_up = False
                if event.key == pygame.K_DOWN:
                    pressed_down = False
                if event.key == pygame.K_LEFT:
                    pressed_left = False
                if event.key == pygame.K_RIGHT:
                    pressed_right = False

        if pressed_up:
            v=v+a*dt
      
        elif pressed_down:
            v=v-a*dt
        else:
            #For smooth deceleration of robot to zero velocity
            if v>0:
                v=v-a*dt
                if v<=0.01:
                    v=0
            elif v<0:
                v=v+a*dt
                if v>=-0.01:
                    v=0

        if pressed_left:
            w=w+al*dt
        elif pressed_right:
            w=w-al*dt
        else:
            #For smooth deceleration of robot to zero velocity
            if w>0:
                w=w-al*dtw
                if w<=0.01:
                    w=0
            elif w<0:
                w=w+al*dtw
                if w>=-0.01:
                    w=0

        if v>=V_MAX:
            v=V_MAX
        elif v<=-V_MAX:
            v=-V_MAX
        if w>=W_MAX:
            w=W_MAX
        elif w<=-W_MAX:
            w=-W_MAX

        vel = velcommand(v,w)
        rospy.loginfo([v,w])
        pub.publish(vel)   #publish the velocity (v,w)
        rate.sleep()


if __name__ == '__main__':
    try:
        speed()
    except rospy.ROSInterruptException:
        pass
