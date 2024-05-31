#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class RoboClawController:
    def __init__(self):
        self.distancia_llantas = 0.8 # Ajusta esta distancia según tu configuración
        self.mode = 0
        self.roboclaw_left_speed = 0
        self.roboclaw_right_speed = 0

        rospy.init_node('roboclaw_controller')
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber("/joy_roboclaw",Joy, self.joy_callback)

    def joy_callback(self, msg):
        for i in range(len(msg.buttons)):
            if msg.buttons[i] == 1:
                self.mode = i
                break
    def cmd_vel_callback(self, msg):
        self.roboclaw_right_speed = msg.linear.x + (self.distancia_llantas / 2) * msg.angular.z
        self.roboclaw_left_speed = msg.linear.x - (self.distancia_llantas / 2) * msg.angular.z

        if self.mode == 5:
            max_speed = 2
        else:
            max_speed = 1

        # rospy.loginfo(f"Max Speed: {max_speed}" )
        
        # if (msg.angular.z == 2.5 or msg.angular.z == -2.5 or msg.angular.z == 1.5 or msg.angular.z == -1.5) and msg.linear.x == 0:
        #     self.roboclaw_right_speed *= 2
        #     self.roboclaw_left_speed *= 2
        if msg.angular.z != 0 and msg.linear.x == 0:
            self.roboclaw_right_speed *= 2
            self.roboclaw_left_speed *= 2
        
        self.roboclaw_left_speed, self.roboclaw_right_speed = self.limit_speed(max_speed, self.roboclaw_left_speed, self.roboclaw_right_speed)

        if self.roboclaw_left_speed !=0 and self.roboclaw_right_speed !=0:
            self.roboclaw_left_speed, self.roboclaw_right_speed = self.limit_low_speed(1, 2, self.roboclaw_left_speed, self.roboclaw_right_speed)

        rospy.loginfo(f"Left Speed: {self.roboclaw_left_speed}" )
        rospy.loginfo(f"Right Speed: {self.roboclaw_right_speed}")


    def limit_speed(self, max, left, right):
        if abs(left) > max:
            left = max
        if abs(right) > max:
            right = max
        return left, right
    
    def limit_low_speed(self, min, max, left, right):
        if abs(left) < min:
            newSpeed = max / left
            if left > 0:
                newSpeed = (max - min) / newSpeed + min
            else:
                newSpeed = (max - min) / newSpeed - min
            left = newSpeed
        if abs(right) < min:
            newSpeed = max / right
            if right > 0:
                newSpeed = (max - min) / newSpeed + min
            else:
                newSpeed = (max - min) / newSpeed - min
            right = newSpeed

        return left, right

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = RoboClawController()
    controller.run()
