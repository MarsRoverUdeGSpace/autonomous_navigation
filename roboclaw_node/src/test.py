import time
from os import system
from roboclaw_3 import Roboclaw

#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name
def reads(rc, address):
    #print("Corriente del motor M1:", rc.ReadISpeedM1(address))
    #print("Corriente del motor M2:", rc.ReadISpeedM2(address))
    print("Corrientes:", rc.ReadCurrents(address))
    print("PWMs:", rc.ReadPWMs(address))
    print("Voltaje de la batería principal:", rc.ReadMainBatteryVoltage(address))
    #print("Voltajes mínimo y máximo de la batería principal:", rc.ReadMinMaxMainVoltages(address))

system("sudo chmod 777 /dev/ttyACM0")
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80
unVolt = int(32767/10.7)
voltajeDeseado = 11
print(unVolt*voltajeDeseado)
rc.DutyAccelM1M2(address, 12000, unVolt*voltajeDeseado, 12000, unVolt*voltajeDeseado)

for i in range(100):
    reads(rc, address)
    time.sleep(0.1)
#rc.BackwardM1(address, 63) # address, val (0-126) M2 tambien
#rc.BackwardM2(address, 63) # address, val (0-126) M2 tambien
rc.DutyAccelM1M2(address,12000, 0, 12000, 0)
for i in range(15):
    reads(rc, address)
    time.sleep(0.1)