#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
import time
import os

os.system ("sudo pigpiod")
time.sleep(1)

import pigpio
import math
import numpy as np

ESC1 = 4
ESC2 = 17
ESC3 = 3
ESC4 = 5

minValue = 1000
maxValue = 2000

pi = pigpio.pi()
pi.set_servo_pulsewidth (ESC1, 0)
pi.set_servo_pulsewidth (ESC2, 0)

time.sleep(1)

pi.set_servo_pulsewidth (ESC1, maxValue)
pi.set_servo_pulsewidth (ESC2, maxValue)

time.sleep(1)

pi.set_servo_pulsewidth (ESC1, minValue)
pi.set_servo_pulsewidth (ESC2, minValue)

time.sleep(1)

global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime
roll = 0.0
pitch = 0.0
yaw = 0.0

pub = rospy.Publisher('QuadMotorTopic', Float64 , queue_size = 4)

def callback1(imu_data):

        global roll, pitch, yaw

        pitch = imu_data.orientation.y
        roll = imu_data.orientation.x
        yaw = imu_data.orientation.z

def callback2(rcReceiver):

        global roll, pitch, yaw
        #Assign your PID values here. From symmetry, control for roll and pitch is the same.

        kp_roll = 70
        ki_roll = 0.0002
        kd_roll = 89
        kp_pitch = kp_roll
        ki_pitch = ki_roll
        kd_pitch = kd_roll
        kp_yaw = 0.1
        ki_yaw = 0
        kd_yaw = 0
        flag = 0

        #Define other variables here, and calculate the errors.

        sampleTime = 0
        setpoint = 0
        err_pitch = float(pitch)*(180 / 3.141592653) - rcReceiver.data[0]
        err_roll = float(roll)*(180 / 3.141592653) - rcReceiver.data[1]
        err_yaw = float(yaw)*(180/3.14159263) - rcReceiver.data[3]
        currTime = time.time()

        #Reset the following variables during the first run only.
        if flag == 0:
                prevTime = 0
                prevErr_roll = 0
                prevErr_pitch = 0
                prevErr_yaw = 0
                pMem_roll = 0
                pMem_pitch = 0
                pMem_yaw = 0
                iMem_roll = 0
                iMem_pitch = 0
                iMem_yaw = 0
                dMem_roll = 0
                dMem_pitch = 0
                dMem_yaw = 0
                flag += 1

        #Define dt, dy(t) here for kd calculations.
        dTime = currTime - prevTime
        dErr_pitch = err_pitch - prevErr_pitch
        dErr_roll = err_roll - prevErr_roll
        dErr_yaw = err_yaw - prevErr_yaw

        #-------------------------------------------------------------------------------------------------------------------------------
        #This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
        if(dTime >= sampleTime):
                #Kp*e(t)
                pMem_roll = kp_roll * err_roll
                pMem_pitch = kp_pitch * err_pitch
                pMem_yaw = kp_yaw * err_yaw

                #integral(e(t))
                iMem_roll += err_pitch * dTime
                iMem_pitch += err_roll * dTime
                iMem_yaw += err_yaw * dTime

                if(iMem_roll > 400): iMem_roll = 400
                if(iMem_roll < -400): iMem_roll = -400
                if(iMem_pitch > 400): iMem_pitch = 400
                if(iMem_pitch < -400): iMem_pitch = -400
                if(iMem_yaw > 400): iMem_yaw = 400
                if(iMem_yaw < -400): iMem_yaw = 400

                #derivative(e(t))
                dMem_roll = dErr_roll / dTime
                dMem_pitch = dErr_pitch / dTime
                dMem_yaw = dErr_yaw / dTime

        #Store the current variables into previous variables for the next iteration.
        prevTime = currTime
        prevErr_roll = err_roll
        prevErr_pitch = err_pitch
        prevErr_yaw = err_yaw
        output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
        output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
        output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 

        #-------------------------------------------------------------------------------------------------------------------------------
                #Ignore this.
        #br_motor_vel = 50.5 + output_pitch + output_roll + output_yaw
        #bl_motor_vel = 50.5 - output_pitch + output_roll - output_yaw
        #fl_motor_vel = 50.5 - output_pitch - output_roll + output_yaw
        #fr_motor_vel = 50.5 + output_pitch - output_roll - output_yaw

        #-------------------------------------------------------------------------------------------------------------------------------
        #Some Gazebo information for your reference.

        #Positive roll is right wing down
        #Positive pitch is front nose down
        #Positive yaw is rotate CCW about z-axis

        #Red is x-axis
        #Green is y-axis
        #Blue is z-axis

        #-------------------------------------------------------------------------------------------------------------------------------
        #br: Back Right
        #bl: Back Left
        #fl: Front Left
        #fr: Front Right
        #Calculate the ESC pulses (1000us - 2000us PWM signal) for each of the motor.

        #br in my code is fr in gazebo's world
        esc_br = rcReceiver.data[2] + output_roll + output_pitch - output_yaw
        #bl in my code is br in gazebo's world
        esc_bl = rcReceiver.data[2] + output_roll - output_pitch + output_yaw
        #fl in my code is bl in gazebo's world
        esc_fl = rcReceiver.data[2] - output_roll - output_pitch - output_yaw
        #fr in my code is fl in gazebo's world
        esc_fr = rcReceiver.data[2] - output_roll + output_pitch + output_yaw

        #Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
        if(esc_br > 2000): esc_br = 2000
        if(esc_bl > 2000): esc_bl = 2000
        if(esc_fr > 2000): esc_fr = 2000
        if(esc_fl > 2000): esc_fl = 2000

        if(esc_br < 1100): esc_br = 1100
        if(esc_bl < 1100): esc_bl = 1100
        if(esc_fr < 1100): esc_fr = 1100
        if(esc_fl < 1100): esc_fl = 1100

        pi.set_servo_pulsewidth(ESC1, esc_fl)
        pi.set_servo_pulsewidth(ESC2, esc_fr)
        pi.set_servo_pulsewidth(ESC3, esc_bl)
        pi.set_servo_pulsewidth(ESC4, esc_br)
        print("ESC_FL: ", esc_fl)
        print("ESC_FR: ", esc_fr)
        print("ESC_BL: ", esc_bl)
        print("ESC_BR: ", esc_br)
#        print("data", rcReceiver.data)
        pub.publish(esc_fl)
        pub.publish(esc_fr)
        pub.publish(esc_br)
        pub.publish(esc_br)


def listener():
        rospy.init_node('QuadMotor', anonymous=True)
        rospy.Subscriber("imu/data", Imu, callback1)
        rospy.Subscriber("rcReceiver", Int16MultiArray, callback2)
        rospy.spin()

if __name__ == '__main__':
        listener()

