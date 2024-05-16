#!/usr/bin/env python3
from os import system
import rospy
#import time
from roboclaw_3 import Roboclaw
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

global max_voltage, voltage_battery
voltage_battery = 21
max_voltage = 14
def limit_speed(max, left, right):
    if left > max:
        left = max
    if right > max:
        right
    return left, right

def cmd_vel_callback(msg):
    global roboclaw_left_speed, roboclaw_right_speed, mode
    distancia_llantas = 0.8
    
    #scale_linear_turbo: 0.7
	#scale_angular_turbo: 0.6
    
    roboclaw_right_speed = msg.linear.x + (distancia_llantas/2)*msg.angular.z
    roboclaw_left_speed = msg.linear.x - (distancia_llantas/2)*msg.angular.z
    if mode == 5:
        max_speed = 2
    elif mode == 0:
        max_speed = 1
    else:
        max_speed = 1
    roboclaw_left_speed, roboclaw_right_speed, = limit_speed(max_speed, roboclaw_left_speed, roboclaw_right_speed)
    if (msg.angular.z == 2.5 or msg.angular.z == -2.5 or msg.angular.z == 1.5 or msg.angular.z == -1.5) and msg.linear.x == 0:
        roboclaw_right_speed *=2
        roboclaw_left_speed *=2
    # test.right = roboclaw_left_speed
    print("Left: ",roboclaw_left_speed)
    print("Right: ",roboclaw_right_speed)
    # rc.DutyAccelM1M2(address, accel1=655359, duty1=1, accel2=655359, duty2=1) # address, accel1 0 to 655359, duty1 -32768 to +32767. accel2 0 to 655359, duty2 -32768 to +32767.


def joy_callback(msg):
    global mode
    for i in range(len(msg.buttons)):
        if msg.buttons[i] == 1:
            mode = i
            break
    # else:
    #     mode = "No hay ningun boton presionado"
    # print(mode)

def main():
    global roboclaw_left_speed, roboclaw_right_speed, mode
    mode = 1
    rospy.init_node("roboclaw_speed")
    print("Waiting for topic /joy_roboclaw")
    print("Waiting for topic /cmd_vel")
    rospy.wait_for_message("/joy_roboclaw",Joy, timeout=50)
    rospy.wait_for_message("/cmd_vel",Twist, timeout=50)
    rospy.Subscriber("/joy_roboclaw",Joy, joy_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    loop = rospy.Rate(3)
    while not rospy.is_shutdown():
        pass
    rospy.spin()

if __name__ == "__main__":
     main()