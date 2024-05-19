#!/usr/bin/env python
from os import system
import rospy
#import time
from roboclaw import Roboclaw
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    global roboclaw_left_speed, roboclaw_right_speed
    distancia_llantas = 0.7
    #scale_linear_turbo: 0.7
	#scale_angular_turbo: 0.6
    
    if abs(msg.linear.x) == 0 and (msg.angular.z == 1 or msg.angular.z == 0.5): #Gira a la izquierda
        left_speed = -(2*abs(msg.angular.z) + 0.375*msg.angular.z)/(0.34) 
        right_speed = (2*abs(msg.angular.z) + 0.375*msg.angular.z)/(0.34)
    elif abs(msg.linear.x) == 0 and (msg.angular.z == -1 or msg.angular.z == -0.5): #Gira a la derecha
        left_speed = (2*abs(msg.angular.z) - 0.375*msg.angular.z)/(0.34)
        right_speed = -(2*abs(msg.angular.z) - 0.375*msg.angular.z)/(0.34)
    else:
        left_speed = (2*msg.linear.x - 0.375*msg.angular.z)/(0.34) 
        right_speed = (2*msg.linear.x + 0.375*msg.angular.z)/(0.34)
    
    roboclaw_left_speed = int((126*left_speed)/((2 + 0.375)/(0.34)))
    roboclaw_right_speed = int((126*right_speed)/((2 + 0.375)/(0.34)))
    if abs(roboclaw_left_speed) == 106 and abs(roboclaw_right_speed) == 106:
        if roboclaw_left_speed > 0:
            roboclaw_right_speed = 126
            roboclaw_left_speed = 126
        else:
            roboclaw_right_speed = -126
            roboclaw_left_speed = -126
    elif (abs(roboclaw_left_speed) == 53 and abs(roboclaw_right_speed) == 53):
        if roboclaw_left_speed > 0:
            roboclaw_right_speed = 63
            roboclaw_left_speed = 63
        else:
            roboclaw_right_speed = -63
            roboclaw_left_speed = -63
    # print("Left: ",roboclaw_left_speed)
    # print("Right: ",roboclaw_right_speed)

def main():
    global left_speed, right_speed
    global roboclaw_left_speed, roboclaw_right_speed

    rospy.init_node("roboclaw_speed")
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    loop = rospy.Rate(3)

    #subprocess.call("sudo chmod 777 /dev/ttyACM0")
    system("sudo chmod 777 /dev/ttyACM0")
    system("sudo chmod 777 /dev/ttyACM1")
    system("sudo chmod 777 /dev/ttyACM2")

    rc1 = Roboclaw('/dev/ttyACM1', 115200)
    rc2 = Roboclaw('/dev/ttyACM2', 115200)
    rc3 = Roboclaw('/dev/ttyACM0', 115200)

    address = 0x80

    rc1.Open()
    rc2.Open()
    rc3.Open()
    #address = 0x80
    rcs = [rc1, rc2, rc3]

    version1 = rc1.ReadVersion(address)
    version2 = rc2.ReadVersion(address)
    version3 = rc3.ReadVersion(address)

    if version1[0] == False:
        print("GETVERSION1 Failed")
    elif version2[0] == False:
        print("GETVERSION2 Failed")
    elif version3[0] == False:
        print("GETVERSION3 Failed")
    else:
        print(repr(version1[1]))
        print(repr(version2[1]))
        print(repr(version3[1]))

    roboclaw_left_speed = 0
    roboclaw_right_speed = 0
    speed = 0
    try:
        while not rospy.is_shutdown():
            print("Left: ",roboclaw_left_speed)
            print("Right: ",roboclaw_right_speed)
            if roboclaw_left_speed > 0 and roboclaw_right_speed > 0:
                # Mover hacia adelante
                print("Adelante o curva adelante")
                print("Izquierda", roboclaw_left_speed)
                print("Derecha", roboclaw_right_speed)
                for rc in rcs:
                    rc.ForwardM1(address, roboclaw_right_speed)
                    rc.ForwardM2(address, roboclaw_left_speed)
                #rc1.ForwardM1(address, roboclaw_right_speed)
                # rc1.ForwardM2(address, roboclaw_left_speed)
                # rc2.ForwardM1(address, roboclaw_right_speed)
                # rc2.ForwardM2(address, roboclaw_left_speed)
                # rc3.ForwardM1(address, roboclaw_right_speed)
                # rc3.ForwardM2(address, roboclaw_left_speed)
            elif roboclaw_left_speed < 0 and roboclaw_right_speed < 0:
                # rc1.BackwardMixed(address, speed)
                # Mover hacia atras
                print("Atras o curva trasera")
                print("Izquierda", roboclaw_left_speed)
                print("Derecha", roboclaw_right_speed)
                for rc in rcs:
                    rc.BackwardM1(address, abs(roboclaw_right_speed))
                    rc.BackwardM2(address, abs(roboclaw_left_speed))
                # rc1.BackwardM1(address, roboclaw_right_speed)
                # rc1.BackwardM2(address, roboclaw_left_speed)
                # rc2.BackwardM1(address, roboclaw_right_speed)
                # rc2.BackwardM2(address, roboclaw_left_speed)
                # rc3.BackwardM1(address, roboclaw_right_speed)
                # rc3.BackwardM2(address, roboclaw_left_speed)
            # Giro a la derecha
            elif roboclaw_left_speed > 0 and roboclaw_right_speed < 0:
                speed = int((abs(roboclaw_left_speed) + abs(roboclaw_right_speed))//2)
                print("Derecha",speed)
                for rc in rcs:
                    rc.TurnRightMixed(address, speed)
                #rc1.TurnRightMixed(address, speed)
                #rc2.TurnRightMixed(address, speed)
                #rc3.TurnRightMixed(address, speed)
            # Giro a la izquierda
            elif roboclaw_left_speed < 0 and roboclaw_right_speed > 0:
                speed = int((abs(roboclaw_left_speed) + abs(roboclaw_right_speed))//2)
                print("Izquierda",speed)
                for rc in rcs:
                    rc.TurnLeftMixed(address,speed)
                #rc1.TurnLeftMixed(address, speed)
                #rc2.TurnLeftMixed(address, speed)
                #rc3.TurnLeftMixed(address, speed)
            # Detener
            else:
                speed = 0
                print("Detener", speed)
                for rc in rcs:
                    rc.ForwardM1(address, speed)
                    rc.ForwardM2(address, speed)
                #rc1.ForwardM1(address, speed)
                #rc1.ForwardMixed(address, speed)
                #rc2.ForwardMixed(address, speed)
                #rc3.ForwardMixed(address, speed)
            loop.sleep()
            print("")
    except:
        speed = 0
        print("Detener", speed)
        for rc in rcs:
            try:
                rc.ForwardM1(address, speed)
                rc.ForwardM2(address, speed)
            except:
                pass
        print("Error")

if __name__ == "__main__":
     main()