#!/usr/bin/env python3
from math import pi, cos, sin

from os import system
import diagnostic_msgs
import diagnostic_updater
from roboclaw_3 import Roboclaw
import classDrive
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom1_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # self.odom2_pub = rospy.Publisher('/odom2', Odometry, queue_size=10)
        # self.odom3_pub = rospy.Publisher('/odom3', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()


    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        # odom.header.frame_id = headerID
        
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        # odom.child_frame_id = childID
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
        # if i == 0:
        self.odom1_pub.publish(odom)
        # if i == 1:
        #     self.odom2_pub.publish(odom)
        # if i == 2:
        #     self.odom3_pub.publish(odom)




class Node:
    def __init__(self):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
        self.drive = classDrive.Drive()

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = str(rospy.get_param("~dev", "/dev/ttyACM0"))
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.address = int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
        
        # TODO need someway to check if address is correct

        try:
            self.drive.openRcs(3,"/dev/ttyACM",baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.loginfo(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        # self.updater.add(diagnostic_updater.
        #                  FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            self.drive.versions()
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
        
        # self.robo_claw.SpeedM1M2(self.address, 0, 0)
        # self.robo_claw.ResetEncoders(self.address)
        self.drive.resetEncoders()

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "1220"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.8"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        # self.encodm1 = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        # self.encodm2 = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        # self.encodm3 = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()
        # rospy.logwarn("Waiting for topic /joy_roboclaw")
        rospy.logwarn("Waiting for topic /cmd_vel")
        # rospy.wait_for_message("/joy_roboclaw",Joy)
        #rospy.wait_for_message("/cmd_vel",Twist)

        rospy.Subscriber("cmd_vel", Twist, self.drive.cmd_vel_callback)
        rospy.Subscriber("/joy_roboclaw",Joy, self.drive.joy_callback)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", self.address)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.drive.roboclaw_left_speed = self.drive.roboclaw_left_speed/2
            self.drive.roboclaw_right_speed = self.drive.roboclaw_right_speed/2
            self.drive.voltageBattery = self.drive.meanVoltage()
            # self.drive.reads()
            dutyMax = (32767/self.drive.voltageBattery)*self.drive.maxVoltage*10
            dutyLeft = int(dutyMax * self.drive.roboclaw_left_speed)
            dutyRight = int(dutyMax * self.drive.roboclaw_right_speed)
            if dutyLeft > 16000:
                dutyLeft = 16000
            if dutyRight > 16000:
                dutyRight = 16000
            # print("Left: ",self.drive.roboclaw_left_speed)
            # print("Right: ",self.drive.roboclaw_right_speed)
            # print("Left: ",dutyLeft)
            # print("Right: ",dutyRight)
            self.drive.pub_data(dutyLeft, dutyRight)
            print("Duty left: ", dutyLeft)
            print("Duty right: ", dutyRight)

            for rc in self.drive.rcs:
                rc.DutyAccelM1M2(self.drive.address, 100000, dutyLeft, 100000, dutyRight)

            # self.robo_claw.DutyAccelM1M2(self.address, 200000, dutyRight, 200000, dutyLeft)
    # # # # # # Aqui me quede
            # TODO need find solution to the OSError11 looks like sync problem with serial
            
            self.encoders(0)
            # for i in range(len(self.drive.rcs)):
            #     self.encoders(i)
            self.updater.update()
            # status1, enc1, crc1 = None, None, None
            # status2, enc2, crc2 = None, None, None

            # try:
            #     status1, enc1, crc1 = self.robo_claw.ReadEncM1(self.address)
            # except ValueError:
            #     pass
            # except OSError as e:
            #     rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
            #     rospy.logdebug(e)

            # try:
            #     status2, enc2, crc2 = self.robo_claw.ReadEncM2(self.address)
            # except ValueError:
            #     pass
            # except OSError as e:
            #     rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
            #     rospy.logdebug(e)

            # if ('enc1' in vars()) and ('enc2' in vars()):
            #     rospy.logdebug(" Encoders %d %d" % (enc1, enc2))
            #     self.encodm.update_publish(enc1, enc2)

            r_time.sleep()

    def encoders(self,i):
        status1, enc1, crc1 = None, None, None
        status2, enc2, crc2 = None, None, None

        try:
            status1, enc1, crc1 = self.drive.rcs[i].ReadEncM1(self.address)
            # rospy.loginfo(enc1)
        except ValueError:
            pass
        except OSError as e:
            rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
            rospy.logdebug(e)
            
        try:
            status2, enc2, crc2 = self.drive.rcs[i].ReadEncM2(self.address)
            # rospy.loginfo(enc2)
        except ValueError:
            pass
        except OSError as e:
            rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
            rospy.logdebug(e)

        if ('enc1' in vars()) and ('enc2' in vars()):
            self.encodm.update_publish(enc1, enc2)
            # rospy.loginfo(" Encoders %d %d" % (enc1, enc2))
            # if i == 0:
            #     self.encodm1.update_publish(enc1, enc2,"odom"+str(i),"base_link"+str(i),i)
            # if i == 1:
            #     self.encodm2.update_publish(enc1, enc2,"odom"+str(i),"base_link"+str(i),i)
            # if i == 2:
            #     self.encodm3.update_publish(enc1, enc2,"odom"+str(i),"base_link"+str(i),i)

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            for rc in self.drive.rcs:
                rc.ForwardM1(self.drive.address,0)
                rc.ForwardM2(self.drive.address,0)
            # self.robo_claw.ForwardM1(self.address, 0)
            # self.robo_claw.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                for rc in self.drive.rcs:
                    rc.ForwardM1(self.drive.address,0)
                    rc.ForwardM2(self.drive.address,0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    # except rospy.ROSInterruptException:
    except Exception:
        print(Exception)
    rospy.loginfo("Exiting")
