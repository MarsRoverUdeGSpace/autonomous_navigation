#!/usr/bin/env python
from os import system
import rospy
import sys
import time
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
    print("Left: ",roboclaw_left_speed)
    print("Right: ",roboclaw_right_speed)

def ports():
    global num_port
    
    try:
        command = "sudo chmod 777 /dev/ttyACM"
        com = system(command+str(num_port))
        if com == 256:
            print(command+str(num_port))
            if num_port == 500:
                print("Se ha sobrepasado el numero de intentos")
                #sys.exit()
                movement()
            num_port += 1
            ports()
        else:
            return "/dev/ttyACM" + str(num_port)
    except:
        num_port += 1
        ports()

def movement():
    global num_port
    num_port = 0
    baud = 115200
    loop = rospy.Rate(3)
    portrc1 = ports()
    portrc2 = ports()
    portrc3 = ports()
    rc1 = Roboclaw(portrc1,baud)
    rc2 = Roboclaw(portrc2,baud)
    rc3 = Roboclaw(portrc3,baud)
    print(portrc1,portrc2,portrc3)
    address = 0x80

    rc1.Open()
    rc2.Open()
    rc3.Open()
    print("Bateria 2:", rc1.ReadMainBatteryVoltage(address))
    print("Bateria 1: ", rc2.ReadMainBatteryVoltage(address))
    #address = 0x80
    rcs = [rc1, rc2, rc3]
    version1 = rc1.ReadVersion(address)
    version2 = rc2.ReadVersion(address)
    version3 = rc2.ReadVersion(address)

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
   
    try:
        while not rospy.is_shutdown():
#            print("Left: ",roboclaw_left_speed)
#            print("Right: ",roboclaw_right_speed)

            if roboclaw_left_speed > 0 and roboclaw_left_speed !=0:
            #print("Left: ",roboclaw_left_speed)
            #print("Right: ",roboclaw_right_speed)
    #	    print("Adelante")
                rc1.ForwardM1(address, int(roboclaw_left_speed))
                rc2.ForwardM1(address, int(roboclaw_left_speed))
                rc3.ForwardM1(address, int(roboclaw_left_speed))
            else:
                if roboclaw_left_speed < 0:
                    print("Izquierda o atras")
                rc1.BackwardM1(address, int(-roboclaw_left_speed))
                rc2.BackwardM1(address, int(-roboclaw_left_speed))
                rc3.BackwardM1(address, int(-roboclaw_left_speed))

            if roboclaw_right_speed > 0 and roboclaw_right_speed != 0:
    #	    print("Izquierda")
                rc1.ForwardM2(address, int(roboclaw_right_speed))
                rc2.ForwardM2(address, int(roboclaw_right_speed))
                rc3.ForwardM2(address, int(roboclaw_right_speed))
            else:
    #	    print("Derecha")
                rc1.BackwardM2(address, int(-roboclaw_right_speed))
                rc2.BackwardM2(address, int(-roboclaw_right_speed))
                rc3.BackwardM2(address, int(-roboclaw_right_speed))
            print("Left: ",roboclaw_left_speed)
            print("Right: ",roboclaw_right_speed)

            loop.sleep()
    except:
        print("AAAA")
        time.sleep(5)
        movement()

    rc1.ForwardM1(address, 0)
    rc1.ForwardM2(address, 0)
    rc2.ForwardM1(address, 0)
    rc2.ForwardM2(address, 0)
    rc3.ForwardM1(address, 0)
    rc3.ForwardM2(address, 0)

def main():
    global num_port
    global roboclaw_left_speed, roboclaw_right_speed
    command = "sudo chmod 777 /dev/ttyACM"

    rospy.init_node("roboclaw_speed")
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    
    movement()


if __name__ == "__main__":
     main()
