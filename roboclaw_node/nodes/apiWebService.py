#!/usr/bin/env python3
from flask import Flask, jsonify
from threading import Thread
import rospy
from sensor_msgs import NavSatFix
from std_msgs.msg import Int16MultiArray

from rospy import init_node as ros_init, Subscriber as ros_subs, spin as ros_spin

# CONFIGURE FLASK
app = Flask(__name__)
app.config.update(
    TESTING=True,
    SECRET_KEY='192b9bdd22ab9ed4d12e236c78afcb9a393ec15f71bbf5dc987d54727823bcbf'
)

# OBJETOS QUE SE EXPORTAN A LA API


def navSatFix_callback(data):
    rospy.loginfo(f"Data: {data}")

def navSatFix_callback(data):
    """
    0 Batería RC1
    1 Corriente M1 RC1
    2 Corriente M2 RC1
    3 Batería RC2
    4 Corriente M1 RC2
    5 Corriente M2 RC2
    6 Batería RC3
    7 Corriente M1 RC2
    8 Corriente M2 RC3
    9 Duty Left
    10 Duty Right
    """
    rospy.loginfo(f"Data: {data}")


def ros_thread():
    ros_init("ApiWebService", anonymous=True)
    ros_subs("/fix", NavSatFix, navSatFix_callback)
    ros_subs("/roboclaw_data", Int16MultiArray, navSatFix_callback)
    ros_spin()

def ros_init_app():
    ros_init("ApiWebService", anonymous=True)
    ros_subs("/fix", NavSatFix, navSatFix_callback)
    ros_spin()

def flask_thread():
    app.run()

def main():
    ros_init_app()
    flask_t = Thread(target=flask_thread)
    ros_t = Thread(target=ros_thread)

    flask_t.start()
    ros_t.start()

    flask_t.join()
    ros_t.join()

if __name__ == "__main__":
    main()