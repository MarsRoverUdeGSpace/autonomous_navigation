#!/usr/bin/env python3
from os import system
import rospy
#import time
from roboclaw_3 import Roboclaw
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

global maxVoltage, voltageBattery
voltageBattery = 21
maxVoltage = 14
def limit_speed(max, left, right):
    if left > max:
        left = max
    if right > max:
        right
    return left, right

def cmd_vel_callback(msg):
    global roboclaw_left_speed, roboclaw_right_speed, mode
    distancia_llantas = 0.8
    
    roboclaw_right_speed = msg.linear.x + (distancia_llantas/2)*msg.angular.z
    roboclaw_left_speed = msg.linear.x - (distancia_llantas/2)*msg.angular.z
    if mode == 5:
        max_speed = 2
    elif mode == 0:
        max_speed = 1
    else:
        max_speed = 1
    roboclaw_left_speed, roboclaw_right_speed = limit_speed(max_speed, roboclaw_left_speed, roboclaw_right_speed)
    if (msg.angular.z == 2.5 or msg.angular.z == -2.5 or msg.angular.z == 1.5 or msg.angular.z == -1.5) and msg.linear.x == 0:
        roboclaw_right_speed *=2
        roboclaw_left_speed *=2

def joy_callback(msg):
    global mode
    for i in range(len(msg.buttons)):
        if msg.buttons[i] == 1:
            mode = i
            break
    # else:
    #     mode = "No hay ningun boton presionado"
    # print(mode)

def reads(address,rcs):
    for rc in rcs:
        print("Battery: ", rc.ReadMainBatteryVoltaje(address))
        print("Current M1: ", rc.ReadCurrents(address)[1])
        print("Current M2: ", rc.ReadCurrents(address)[2])


def openRcs():
    rcs = []
    for i in range(3):
        rc = Roboclaw("/dev/ttyACM"+str(i), 115200)
        rc.Open()
        rcs.append(rc)
    return rcs
def permisos():
    system("sudo chmod 777 /dev/ttyACM0")
    system("sudo chmod 777 /dev/ttyACM1")
    system("sudo chmod 777 /dev/ttyACM2")

def versions(rcs, address):
    version1 = rcs[0].ReadVersion(address)
    version2 = rcs[1].ReadVersion(address)
    version3 = rcs[2].ReadVersion(address)

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

def meanVoltage(rcs, address):
    prom_voltage = 0
    for i in range(len(rcs)):
        prom_voltage += rcs[i].ReadMainBatteryVoltage(address)[1]
        return int(prom_voltage/len(rcs))
def main():
    global roboclaw_left_speed, roboclaw_right_speed, mode
    permisos()
    rospy.init_node("roboclaw_speed")
    print("Waiting for topic /joy_roboclaw")
    print("Waiting for topic /cmd_vel")
    rospy.wait_for_message("/joy_roboclaw",Joy)
    rospy.wait_for_message("/cmd_vel",Twist)
    rospy.Subscriber("/joy_roboclaw",Joy, joy_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    loop = rospy.Rate(3)

    mode = 1
    address = 0x80
    rcs = openRcs()
    versions(rcs,address)
    roboclaw_left_speed = 0
    roboclaw_right_speed = 0

    try:
        while not rospy.is_shutdown():
            roboclaw_left_speed = roboclaw_left_speed/2
            roboclaw_right_speed = roboclaw_right_speed/2
            voltageBattery = meanVoltage(rcs,address)
            dutyMax = (32767/voltageBattery)*maxVoltage*10
            dutyLeft = int(dutyMax * roboclaw_left_speed)
            dutyRight = int(dutyMax * roboclaw_right_speed)
            print("Left: ",roboclaw_left_speed)
            print("Right: ",roboclaw_right_speed)
            # print("Left: ",dutyLeft)
            # print("Right: ",dutyRight)
            
            for rc in rcs:
                rc.DutyAccelM1M2(address, 200000, dutyRight, 200000, dutyLeft)
            loop.sleep()
    except Exception:
        print("Detener", 0)
        for rc in rcs:
            try:
                rc.ForwardM1(address, 0)
                rc.ForwardM2(address, 0)
            except Exception:
                print(Exception)
        print(Exception)

if __name__ == "__main__":
     main()