#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import nn_wind_estimator_func as nn
import math
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import tf

global pitot_pressure, odom, pub

def transf_matrix(phi, theta, psi):
    S = np.array([[math.cos(psi)*math.cos(theta), math.sin(psi)*math.cos(theta), -math.sin(theta)],
        [math.cos(psi)*math.sin(theta)*math.sin(phi)-math.sin(psi)*math.cos(phi), math.sin(psi)*math.sin(theta)*math.sin(phi)+math.cos(psi)*math.cos(phi), math.cos(theta)*math.sin(phi)],
        [math.cos(psi)*math.sin(theta)*math.cos(phi)+math.sin(psi)*math.sin(phi), math.sin(psi)*math.sin(theta)*math.cos(phi)-math.cos(psi)*math.sin(phi), math.cos(theta)*math.cos(phi)]])
    return S


def run_nn_estimator():
    global pitot_pressure, odom
    air_rro = 1.22
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll, -pitch, -yaw+M_PI/2
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    # origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    # Sx = tf.rotation_matrix(roll, xaxis)
    # Sy = tf.rotation_matrix(pitch, yaxis)
    # Sz = tf.rotation_matrix(yaw, zaxis)
    S = tf.transformations.euler_matrix(yaw, pitch, roll, 'rzyx')

    # S = transf_matrix(roll,pitch,yaw)
    Vg = np.array([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y], [odom.twist.twist.linear.z],[1]])
    Venu= S.dot(Vg)
    
    # print(Venu)

    ua = math.sqrt(pitot_pressure)
    # ua2_inp = math.pow(ua,2)
    Vd2_inp = math.pow(-Venu[2][0],2)
    Vn_inp = Venu[1][0]
    Vn2_inp = math.pow(Venu[1][0],2)
    Ve_inp = Venu[0][0]
    Ve2_inp = math.pow(Venu[0][0],2)
    Van_aprox = ua*math.cos(-pitch)*math.cos(-yaw+3.141592653/2)
    Vae_aprox = ua*math.cos(-pitch)*math.sin(-yaw+3.141592653/2)
    nn_inputs = np.array([[pitot_pressure],[Vd2_inp],[Vn_inp],[Vn2_inp],[Ve_inp],[Ve2_inp],[Van_aprox],[Vae_aprox]])
    nn_outputs = nn.nn_wind_estimator_func(nn_inputs)

    windSpeed = Vector3Stamped()
    windSpeed.vector.x=nn_outputs[0][0];
    windSpeed.vector.y=nn_outputs[1][0];
    windSpeed.vector.z=nn_outputs[2][0];
    windSpeed.header.stamp = rospy.Time.now();
    windSpeed.header.frame_id = "base_link";
    pub.publish(windSpeed)




def odomCallback(odom_msg):
    global odom
    odom = odom_msg
    run_nn_estimator()

def pitotCallback(pitot_msg):
    global pitot_pressure
    pitot_pressure = pitot_msg.data



def wind_estimator_nn():
    global pub, pitot_pressure
    rospy.init_node('wind_estimator_nn', anonymous=True)

    pub = rospy.Publisher('wind_speed/filtered_nn', Vector3Stamped, queue_size=10)
    rospy.Subscriber("/odometry/filtered", Odometry, odomCallback)
    rospy.Subscriber("/droni_sensors/pitot_pressure", Float64, pitotCallback)
    
    pitot_pressure=0
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        wind_estimator_nn()
    except rospy.ROSInterruptException:
        pass

