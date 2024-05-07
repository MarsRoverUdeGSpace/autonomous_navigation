#!/usr/bin/env python3
from os import system
import rospy
#import time
from roboclaw_3 import Roboclaw
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    global roboclaw_left_speed, roboclaw_right_speed
    distancia_llantas = 0.8
    #scale_linear_turbo: 0.7
	#scale_angular_turbo: 0.6
    
    roboclaw_right_speed = msg.linear.x + (0.8/2)*msg.angular.z
    roboclaw_left_speed = msg.linear.x - (0.8/2)*msg.angular.z
    # if abs(msg.linear.x) == 0 and (msg.angular.z == 1 or msg.angular.z == 0.5): #Gira a la izquierda
    #     left_speed = -(2*abs(msg.angular.z) + 0.8*msg.angular.z)/(0.34) 
    #     right_speed = (2*abs(msg.angular.z) + 0.8*msg.angular.z)/(0.34)
    # elif abs(msg.linear.x) == 0 and (msg.angular.z == -1 or msg.angular.z == -0.5): #Gira a la derecha
    #     left_speed = (2*abs(msg.angular.z) - 0.8*msg.angular.z)/(0.34)
    #     right_speed = -(2*abs(msg.angular.z) - 0.8*msg.angular.z)/(0.34)
    # else:
    #     left_speed = (2*msg.linear.x - 0.8*msg.angular.z)/(0.34) 
    #     right_speed = (2*msg.linear.x + 0.8*msg.angular.z)/(0.34)
    
    # roboclaw_left_speed = int((126*left_speed)/((2 + 0.8)/(0.34)))
    # roboclaw_right_speed = int((126*right_speed)/((2 + 0.8)/(0.34)))

    print("Left: ",roboclaw_left_speed)
    print("Right: ",roboclaw_right_speed)

def main():
    global roboclaw_left_speed, roboclaw_right_speed

    rospy.init_node("roboclaw_speed")
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    loop = rospy.Rate(3)
    while not rospy.is_shutdown():
        rospy.spin()
    #subprocess.call("sudo chmod 777 /dev/ttyACM0")
    # system("sudo chmod 777 /dev/ttyACM0")
    # system("sudo chmod 777 /dev/ttyACM1")
    # system("sudo chmod 777 /dev/ttyACM2")

    # rc1 = Roboclaw('/dev/ttyACM1', 115200)
    # rc2 = Roboclaw('/dev/ttyACM2', 115200)
    # rc3 = Roboclaw('/dev/ttyACM0', 115200)

    # address = 0x80

    # rc1.Open()
    # rc2.Open()
    # rc3.Open()
    # #address = 0x80
    # rcs = [rc1, rc2, rc3]

    # version1 = rc1.ReadVersion(address)
    # version2 = rc2.ReadVersion(address)
    # version3 = rc3.ReadVersion(address)

    # if version1[0] == False:
    #     print("GETVERSION1 Failed")
    # elif version2[0] == False:
    #     print("GETVERSION2 Failed")
    # elif version3[0] == False:
    #     print("GETVERSION3 Failed")
    # else:
    #     print(repr(version1[1]))
    #     print(repr(version2[1]))
    #     print(repr(version3[1]))

    # dutyMaxSpeed = 32767
    # roboclaw_left_speed = 0
    # roboclaw_right_speed = 0
    # left_speed = 0
    # right_speed = 0
    # speed = 0
    # try:
    #     while not rospy.is_shutdown():
    #         print("Left: ",roboclaw_left_speed)
    #         print("Right: ",roboclaw_right_speed)
    #         left_speed = int(dutyMaxSpeed/126)*roboclaw_left_speed
    #         right_speed = int(dutyMaxSpeed/126)*roboclaw_right_speed
    #         for i in rcs:
    #             rcs[i].DutyAccelM1M2(address, 12000, right_speed, 12000, left_speed)
    #         loop.sleep()
    #         print("")
    # except:
    #     speed = 0
    #     print("Detener", speed)
    #     for rc in rcs:
    #         try:
    #             rc.ForwardM1(address, speed)
    #             rc.ForwardM2(address, speed)
    #         except:
    #             pass
    #     print("Error")

if __name__ == "__main__":
     main()