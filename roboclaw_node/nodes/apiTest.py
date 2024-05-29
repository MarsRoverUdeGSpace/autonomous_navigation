#!/usr/bin/env python3
from flask import Flask, jsonify
from threading import Thread
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# CONFIGURE FLASK
app = Flask(__name__)
app.config.update(
    TESTING=True,
    SECRET_KEY='192b9bdd22ab9ed4d12e236c78afcb9a393ec15f71bbf5dc987d54727823bcbf'
)
# todo crear un objeto por cada cosa que quiera mandar
test = {
    "right": 0,
    "left": 0
}
# Objetos nuevos

# 
roboclaw_left_speed = 0
roboclaw_right_speed = 0
mode = 1

def limit_speed(max, left, right):
    if left > max:
        left = max
    if right > max:
        right = max
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
        roboclaw_right_speed *= 2
        roboclaw_left_speed *= 2

    # ASi se acutaliza el objeto    
    test["left"] = roboclaw_left_speed
    test["right"] = roboclaw_right_speed

def joy_callback(msg):
    global mode
    for i in range(len(msg.buttons)):
        if msg.buttons[i] == 1:
            mode = i
            break

def ros_thread():
    rospy.init_node("roboclaw_speed")
    print("Waiting for topic /joy_roboclaw")
    print("Waiting for topic /cmd_vel")
    rospy.wait_for_message("/joy_roboclaw", Joy, timeout=50)
    rospy.wait_for_message("/cmd_vel", Twist, timeout=50)
    rospy.Subscriber("/joy_roboclaw", Joy, joy_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    loop = rospy.Rate(3)
    while not rospy.is_shutdown():
        pass
    rospy.spin()

def ros_init():
    rospy.init_node("roboclaw_speed")
    print("Waiting for topic /joy_roboclaw")
    print("Waiting for topic /cmd_vel")
    rospy.wait_for_message("/joy_roboclaw", Joy, timeout=50)
    rospy.wait_for_message("/cmd_vel", Twist, timeout=50)


@app.route('/test/get')
def loquesea():
    try:
        return jsonify(test)
    except Exception as e:
        return str(e)

def flask_thread():
    app.run()

def main():
    ros_init()
    flask_t = Thread(target=flask_thread)
    ros_t = Thread(target=ros_thread)

    flask_t.start()
    ros_t.start()

    flask_t.join()
    ros_t.join()

if __name__ == "__main__":
    main()
