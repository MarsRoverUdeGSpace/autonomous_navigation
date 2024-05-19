import time
from roboclaw import Roboclaw
from os import system
#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name
rc1 = Roboclaw("/dev/ttyACM0",115200)
rc2 = Roboclaw("/dev/ttyACM0",115200)
rc3 = Roboclaw("/dev/ttyACM0",115200)
rcs = [rc1, rc2,rc3]
for rc in rcs:
    rc.open()

address = 0x80

def reads(rc, address):
    print("Encoder M1:", rc.ReadEncM1(address))
    print("Encoder M2:", rc.ReadEncM2(address))
    # print("Voltaje de la batería principal:", rc.ReadMainBatteryVoltage(address))
    # print("Corrientes:", rc.ReadCurrents(address))
    # print("PWMs:", rc.ReadPWMs(address))
system("sudo chmod 777 /dev/ttyACM0")
system("sudo chmod 777 /dev/ttyACM1")
system("sudo chmod 777 /dev/ttyACM2")

rc1 = Roboclaw("/dev/ttyACM0",115200)
rc2 = Roboclaw("/dev/ttyACM1",115200)
rc3 = Roboclaw("/dev/ttyACM2",115200)

address = 0x80
bat1 = rc1.ReadMainBatteryVoltage(address)
bat2 = rc2.ReadMainBatteryVoltage(address)
bat3 = rc3.ReadMainBatteryVoltage(address)

voltajeDeseado = 6
vel1 = int(32767/bat1) * voltajeDeseado
vel2 = int(32767/bat2) * voltajeDeseado
vel3 = int(32767/bat3) * voltajeDeseado
vel = [vel1, vel2, vel3]
for i in range(3):
    print("Vel: ",vel[i])
    rcs[i].DutyAccelM1M2(address, 12000, vel[i], 12000, vel[i]) # address, accel1 0 to 655359, duty1 -32768 to +32767. accel2 0 to 655359, duty2 -32768 to +32767.

for i in range(20):
    reads(rc, address)
    time.sleep(0.1)
    
for rc in rcs:
    rc.DutyAccelM1M2(address,12000, 0, 12000, 0)

for i in range(3):
    print("Vel: ",vel[i])
    rcs[i].DutyAccelM1M2(address, 12000, vel[i], 12000, vel[i]) # address, accel1 0 to 655359, duty1 -32768 to +32767. accel2 0 to 655359, duty2 -32768 to +32767.


for i in range(15):
    reads(rc, address)
    time.sleep(0.1)