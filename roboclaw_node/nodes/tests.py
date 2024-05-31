import time
from roboclaw_3 import Roboclaw

#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80
# rc.ForwardM1(address, 73)
# enc1 = rc.ReadEncM1(address)
# rc.SetM1MaxCurrent(address, 900)
# time.sleep(2)
# #rc.SetMaxVoltageMainBattery(address, 12)
# max_voltage = rc.ReadMinMaxMainVoltages(address)

# current = rc.ReadCurrents(address)
# max_current = rc.ReadM1MaxCurrent(address)
# print("Max Voltage: ",max_voltage)
# print("Encoders: ",enc1)
# print("Current: ",current)
# print("Max Current: ",max_current)
# time.sleep(2)
# rc.ForwardM1(address, 0)


# Sets
#rc.SetEncM1(address,cnd) # address, cnt (no sé que es)
# rc.SetMinVoltageMainBattery(address, 8.25) # address, val (6-34V)
# rc.SetMaxVoltageMainBattery(address, 12.6) # address, val (6-34V)
# rc.SetMinVoltageLogicBattery(address, 8.25) # address, val (6-34V)
# rc.SetMaxVoltageLogicBattery(address, 12.6) # address, val (6-34V)
# #rc.SetM1VelocityPID(address, p, i, d, qpps) # tambien M2 .look into https://python-roboclaw.readthedocs.io/en/latest/api.html#roboclaw.roboclaw.Roboclaw.read_encoder_m1
# rc.SetMainVoltages(address, min=60, max=340) # address, 10 = 1V
# rc.SetLogicVoltages(address, min=60, max=340) # address, 10 = 1V

# Reads
# rc.ReadEncM1(address) # address M2 tambien
# rc.ReadMainBatteryVoltage(address) # address
# rc.ReadLogicBatteryVoltage(address) # address
# rc.ReadSpeedM1(address) # address M2 tambien
# rc.ReadVersion(address) # address
# rc.ReadISpeedM1(address), #address M2 tambien
# rc.ReadBuffers(address) # address
# rc.ReadPWMs(address) # address
# rc.ReadCurrents(address) # address
# rc.ReadM1VelocityPID(address) # address M2 tambien
# rc.ReadMinMaxMainVoltages(address) # address
# rc.ReadMinMaxLogicVoltages(address) # addresss
# rc.ReadEncoderModes(address)

# rc.ResetEncoders(address)

# Movements
# rc.ForwardM1(address, 126) # address, val (0-126) M2 tambien
# rc.BackwardM1(address, 126) # address, val (0-126) M2 tambien
# rc.ForwardBackwardM1(address, 64) # address, val (0-127) M2 tambien. 64 stop, 0 max speed reverse, 127 max speed forward
# rc.ForwardMixed(address, 126) # address, val (0-126). Mixed mode
# rc.BackwardMixed(address, 126) # address, val (0-126). Mixed mode
# rc.TurnRightMixed(address, 126) # address, val (0-126).Mixed mode
# rc.TurnLeftMixed(address, 126) # address, val (0-126).Mixed mode
# rc.DutyM1(address,32767) #address, val(-32767 - 32767) M2 tambien
# rc.DutyM1M2(address,1,1) #address, valM1(-32767 - 32767) valM2(-32767 - 32767)
# rc.SpeedM1(address,1200) # addres, val(-2147483647, 2147483647) M2 tambien
# rc.SpeedM1M2(address,1200,1200) # addres, valM1(-2147483647, 2147483647) valM2(-2147483647, 2147483647)
# rc.SpeedAccelM1()
# rc.SpeedAccelM1M2()
# rc.SpeedDistanceM1()
# rc.SpeedDistanceM1M2()
# rc.SpeedAccelDistanceM1()
# rc.SpeedAccelDistanceM1M2()
# rc.SpeedAccelM1M2_2()
# rc.SpeedAccelDistanceM1M2_2()
# rc.DutyAccelM1(address, accel=655359, duty=1) # address, accel 0 to 655359, duty -32768 to +32767. M2 tambien
# rc.DutyAccelM1M2(address, accel1=655359, duty1=1, accel2=655359, duty2=1) # address, accel1 0 to 655359, duty1 -32768 to +32767. accel2 0 to 655359, duty2 -32768 to +32767.

def all_reads(rc, address):
    print("Encoder M1:", rc.ReadEncM1(address))
    print("Encoder M2:", rc.ReadEncM2(address))
    print("Voltaje de la batería principal:", rc.ReadMainBatteryVoltage(address))
    print("Voltaje de la batería lógica:", rc.ReadLogicBatteryVoltage(address))
    print("Velocidad M1:", rc.ReadSpeedM1(address))
    print("Versión:", rc.ReadVersion(address))
    print("Corriente del motor M1:", rc.ReadISpeedM1(address))
    print("Buffers:", rc.ReadBuffers(address))
    print("PWMs:", rc.ReadPWMs(address))
    print("Corrientes:", rc.ReadCurrents(address))
    print("PID de velocidad M1:", rc.ReadM1VelocityPID(address))
    print("Voltajes mínimo y máximo de la batería principal:", rc.ReadMinMaxMainVoltages(address))
    print("Voltajes mínimo y máximo de la batería lógica:", rc.ReadMinMaxLogicVoltages(address))
    print("Encoders modo", rc.ReadEncoderModes(address))

# rc.DutyM1M2(address, 32767, 32767)
# time.sleep(2)
# rc.DutyM1M2(address, 0, 0)
rc.DutyAccelM1M2(address, 65535, 32767, 65535, 32767)
time.sleep(2)
# rc.DutyAccelM1M2(address, 65535, -32767, 65535, -32767)
# time.sleep(2)
rc.DutyAccelM1M2(address,65535, 0, 65535, 0)
rc.SetM2EncoderMode(address, 128)
# all_reads(rc, address)
# rc.ResetEncoders(address)
# #rc.SpeedAccelDistanceM1
# rc.SpeedAccelM1M2(address,accel=1425, speed1=1425, speed2=1425)
# #rc.SpeedAccelDistanceM1M2(address, 1425*1, 1425, 3000, 1425, 3000, 1)
# all_reads(rc, address)
# time.sleep(5)
# rc.SpeedAccelM1M2(address, 1425*5, 0, 0)
# # 1425 pulses per revolution

