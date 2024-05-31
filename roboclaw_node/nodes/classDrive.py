#!/usr/bin/env python3
from os import system
import rospy
#import time
from roboclaw_3 import Roboclaw
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

class Drive:
    def __init__(self):
        self.roboclaw_left_speed = 0
        self.roboclaw_right_speed = 0
        self.mode = 1
        self.maxVoltage = 15
        self.voltageBattery = 25
        self.rcs = []
        self.distancia_llantas = 0.8
        self.address = 0x80
        self.pub = rospy.Publisher("/roboclaw_data", Int16MultiArray, queue_size=10)


    def pub_data(self, left, rigth):
        self.message_pub = Int16MultiArray()
        self.message_pub.data = []
        for rc in self.rcs:
            status , currentM1, currentM2 = rc.ReadCurrents(self.address)

            self.message_pub.data.append(rc.ReadMainBatteryVoltage(self.address)[1])
            self.message_pub.data.append(currentM1)
            self.message_pub.data.append(currentM2)
        
        self.message_pub.data.append(left)
        self.message_pub.data.append(rigth)
        # print(self.message_pub.data)
        self.pub.publish(self.message_pub)

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
    def joy_callback(self, msg):
        for i in range(len(msg.buttons)):
            if msg.buttons[i] == 1:
                self.mode = i
                break
            # else:
            #     mode = "No hay ningun boton presionado"
            # print(mode)
    def openRcs(self, n, devName, baudRate):
        self.permisos(n)
        for i in range(n):
            rc = Roboclaw(str(devName)+str(i), int(baudRate))
            rc.Open()
            self.rcs.append(rc)

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
            self.roboclaw_left_speed, self.roboclaw_right_speed = self.limit_low_speed(1.5, 2, self.roboclaw_left_speed, self.roboclaw_right_speed)

        rospy.loginfo(f"Left Speed: {self.roboclaw_left_speed}" )
        rospy.loginfo(f"Right Speed: {self.roboclaw_right_speed}")

    def reads(self):
        for rc in self.rcs:
            print("Battery: ", rc.ReadMainBatteryVoltaje(self.address))
            print("Current M1: ", rc.ReadCurrents(self.address)[1])
            print("Current M2: ", rc.ReadCurrents(self.address)[2])
    
    def versions(self):
        for i in range(len(self.rcs)):
            # version.append(self.rcs[i].ReadVersion(self.address))
            # print(self.rcs[i].ReadVersion(self.address))
            if self.rcs[i].ReadVersion(self.address)[1] == False:
                print("GETVERSION1 Failed")
            else:
                print(repr(self.rcs[i].ReadVersion(self.address)[1]))
    
    def meanVoltage(self):
        prom_voltage = 0
        for i in range(len(self.rcs)):
            prom_voltage += self.rcs[i].ReadMainBatteryVoltage(self.address)[1]
        return int(prom_voltage/len(self.rcs))
        
    def permisos(self, n):
        for i in range(n):
            command = "sudo chmod 777 /dev/ttyACM" + str(i)
            system(command)
        
    def resetEncoders(self):
        for rc in self.rcs:
            rc.SpeedM1M2(self.address, 0, 0)
            rc.ResetEncoders(self.address)

def main():
    drive = Drive()
    # drive.permisos(3)
    rospy.init_node("roboclaw_speed")
    print("Waiting for topic /joy_roboclaw")
    print("Waiting for topic /cmd_vel")
    # rospy.wait_for_message("/joy_roboclaw",Joy)
    # rospy.wait_for_message("/cmd_vel",Twist)
    rospy.Subscriber("/joy_roboclaw",Joy, drive.joy_callback)
    rospy.Subscriber("/cmd_vel", Twist, drive.cmd_vel_callback)
    loop = rospy.Rate(3)

    drive.openRcs(3,"/dev/ttyACM",115200)
    drive.versions()

    try:
        while not rospy.is_shutdown():
            drive.roboclaw_left_speed = drive.roboclaw_left_speed/2
            drive.roboclaw_right_speed = drive.roboclaw_right_speed/2
            drive.voltageBattery = drive.meanVoltage()
            dutyMax = (32767/drive.voltageBattery)*drive.maxVoltage*10
            dutyLeft = int(dutyMax * drive.roboclaw_left_speed)
            dutyRight = int(dutyMax * drive.roboclaw_right_speed)
            print("Left: ",drive.roboclaw_left_speed)
            print("Right: ",drive.roboclaw_right_speed)
            # print("Left: ",dutyLeft)
            # print("Right: ",dutyRight)
            
            for rc in drive.rcs:
                rc.DutyAccelM1M2(drive.address, 20000, dutyRight, 20000, dutyLeft)
            loop.sleep()
    except Exception:
        print("Detener", 0)
        for rc in drive.rcs:
            try:
                rc.ForwardM1(drive.address, 0)
                rc.ForwardM2(drive.address, 0)
            except Exception:
                print(Exception)
        print(Exception)

if __name__ == "__main__":
    try:
        main()
    except Exception:
        print(Exception)
    rospy.loginfo("Exiting")
