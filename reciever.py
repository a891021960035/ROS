#!/usr/bin/python3
import rospy
from ntu_racing.msg import arduino_decode1_msg
from ntu_racing.msg import arduino_decode2_msg
from ntu_racing.msg import motor_control_unit_msg
from ntu_racing.msg import battery_management_system_msg
from ntu_racing.msg import inertial_measurement_unit_msg

import os,sys
import time
import math

import numpy as np
import can
import RPi.GPIO as GPIO

from GPS_stuff import GPS_serial_setup, GPS_read
from IP_stuff import Send_IP_info


rospy.init_node('reciever_node')
arduino1_pub = rospy.Publisher("arduino1", arduino_decode1_msg, queue_size=10)
arduino2_pub = rospy.Publisher("arduino2", arduino_decode2_msg, queue_size=10)
MCU1_pub = rospy.Publisher("motor_control_unit1", motor_control_unit_msg, queue_size=10)
MCU2_pub = rospy.Publisher("motor_control_unit2", motor_control_unit_msg, queue_size=10)
BMS_pub = rospy.Publisher("battery_management_system", battery_management_system_msg, queue_size=10)
IMU_pub = rospy.Publisher("inertial_measurement_unit", inertial_measurement_unit_msg, queue_size=10)


# Led_Pin = 12                 #Brake Pin
# Vcu_shutdown_Pin = 32
# WarningLED1_Pin = 29         #Canbus fail
# WarningLED2_Pin = 31         #different accpedal signal
# Brake_sig_Pin = 36           #for BSPD
# Bat_shutdown_Pin = 15        #overheated
# Pump_control_Pin = 12        #turn on pump when over critical temp

# tempL = 0
# tempH = 0
# SOC = 0
# max_V = 0
# min_V = 0
# critical_temp = 35			 #degC
# tire_radius = 0.225			 #m
# rear_car_speed = 0			 #m/s
# front_car_speed = 0			 #m/s
# pedal_adjust_const = 0.0	 #when slip happen, hold back the acc_pedal value


def ALL_GPIO_control(pin, state):
    if state == 1:
        GPIO.output(pin, GPIO.HIGH)
    elif state == 0:
        GPIO.output(pin, GPIO.LOW)

def dectobin(num):
    num = int(num)
    mid = np.zeros(8)
    t = 0
    while True:
        if num == 0:
            break
        num, rem = divmod(num, 2)
        mid[t] = (rem)
        t = t+1
    # mid[0] is LSB, mid[7] is MSB
    return mid

class Arduino:

    @staticmethod
    def decode_1(msg):
        acc_pedal1 = msg.data[0]*0.39
        acc_pedal2 = msg.data[1]*0.39
        brake_pedal1 = msg.data[2]*0.39  
        # 0.39%/bit
        brake_pedal2 = msg.data[3]*0.39
        fl_wheel_angspeed = (msg.data[4]*256 + msg.data[5]*1) * 0.002   
        # 0.002rad/s/bit
        fr_wheel_angspeed = (msg.data[6]*256 + msg.data[7]*1) * 0.002   
        # 0.002rad/s/bit

        front_car_speed = (fl_wheel_angspeed+fr_wheel_angspeed)*tire_radius*math.pi*3.6*2
        front_car_speedL = front_car_speed % 256
        front_car_speedH = int(front_car_speed / 256)
        
        print('acc 1 : '+ acc_pedal1*0.39 +'%\nacc 2 : '+ acc_pedal2*0.39 +'%')
        print('fl_wheel_angspeed : '+ fl_wheel_angspeed +'\nfront_car_speedL : '+ front_car_speedL +'\nfront_car_speedH : 'front_car_speedH)

        if brake_pedal1 > 0 or brake_pedal2 > 0:
            ALL_GPIO_control(Led_Pin, 1)
        else:
            ALL_GPIO_control(Led_Pin, 0)

    	arduino1_data = arduino_decode1_msg()
    	arduino_decode1_msg.currenttime = time.time()
        arduino1_data.acc_pedal1 = acc_pedal1
        arduino1_data.acc_pedal2 = acc_pedal2
        arduino1_data.brake_pedal1 = brake_pedal1
        arduino1_data.brake_pedal2 = brake_pedal2
        arduino1_data.fl_wheel_angspeed = fl_wheel_angspeed
        arduino1_data.fr_wheel_angspeed = fr_wheel_angspeed
        arduino1_pub.publish(arduino1_data)

        return acc_pedal1, brake_pedal1, fr_wheel_angspeed, fl_wheel_angspeed, front_car_speedL, front_car_speedH

    @staticmethod
    def decode_2(msg):
        steer_angle = msg.data[0]*100/255   
        #reck:0–>left 100–>right
        rl_wheel_angspeed = (msg.data[2]*256+msg.data[3]*1) * 0.002  
        # 0.002rad/s/bit
        rr_wheel_angspeed = (msg.data[4]*256+msg.data[5]*1) * 0.002  
        # 0.002rad/s/bit
        rear_avg_wheel_angspeed = (rl_wheel_angspeed + rr_wheel_angspeed)/2
        rear_car_speed = rear_avg_wheel_angspeed * tire_radius 
        #m/s

        arduino2_data = arduino_decode2_msg()
        arduino2_data.currenttime = time.time()
        arduino2_data.steer_angle = steer_angle
        arduino2_data.rl_wheel_angspeed = rl_wheel_angspeed
        arduino2_data.rr_wheel_angspeed = rr_wheel_angspeed
        arduino2_pub.publish(arduino2_data)

        return steer_angle, rr_wheel_angspeed, rl_wheel_angspeed

class MCU:

    @staticmethod
    def state1(msg,n):
        motor_T = msg.data[6]
        controller_T = msg.data[7]

        MCU_data1 = motor_control_unit_msg()
        MCU_data1.currenttime = time.time()
        MCU_data1.motor_T = motor_T
        MCU_data1.controller_T = controller_T
        MCU_data1.num = n
        MCU_data1.state = 1
        MCU_pub.publish(MCU_data1)

    @staticmethod
    def state2(msg,n):
        battery_V = (msg.data[0]*256+msg.data[1]*1) / 128  
        #1/128 V/bit
        battery_I = (msg.data[2]*256+msg.data[3]*1)
        motor_I = (msg.data[4]*256+msg.data[5])
        motor_speed = (msg.data[6]*256+msg.data[7]*1)
        car_speed = motor_speed / 60 * 2 * 3.14 * tire_radius
        car_speedL = car_speed % 256
        car_speedH = car_speed / 256

        if battery_I > 32768:
            battery_I = (battery_I-32768) / 16
        else:
            battery_I = battery_I / 16
        
        if motor_I > 32768:
            motor_I = (motor_I-32768) / 16
        else:
            motor_I = motor_I / 16

        MCU_data2 = motor_control_unit_msg()
        MCU_data2.currenttime = time.time()
        MCU_data2.battery_V = battery_V
        MCU_data2.battery_I = battery_I
        MCU_data2.motor_speed = motor_speed
        MCU_data2.car_speed = car_speed
        MCU_data2.num = n
        MCU_data2.state = 2
        MCU_pub.publish(MCU_data2)
        
        return car_speedL, car_speedH

class BMS:

    @staticmethod
    def state(msg):
        Pack_voltage = (msg.data[0]*256+msg.data[1]) * 0.01 
        # V
        SOC = msg.data[2]
        Max_CellV = (msg.data[3]*256+msg.data[4]) * 0.1 
        #mV
        Min_CellV = (msg.data[5]*256+msg.data[6]) * 0.1 
        #mV
        debyte8 = dectobin(msg.data[7])

        for i in range(8):
            if debyte8[i] == 1:
                print(err_dict[i])

        if debyte8[7] == 1:
            ALL_GPIO_control(Bat_shutdown_Pin, 1)
        elif debyte8[7] == 0:
            ALL_GPIO_control(Bat_shutdown_Pin, 0)

        Pack_voltageL = Pack_voltage % 256
        Pack_voltageH = Pack_voltage / 256

        BMS_data1 = battery_management_system_msg()
        BMS_data1.currenttime = time.time()
        BMS_data1.Pack_voltage = Pack_voltage
        BMS_data1.SOC = SOC
     	BMS_data1.mode = 1
        BMS_pub.publish(BMS_data1)

        return Pack_voltageL, Pack_voltageH, SOC

    @staticmethod
    def pack(msg):
        Pack1_T = msg.data[0] * 1 - 50     #1 degree c/bit offset 50 
        Pack2_T = msg.data[1] * 1 - 50     #1 degree c/bit offset 50
        Pack3_T = msg.data[2] * 1 - 50     #1 degree c/bit offset 50
        Pack4_T = msg.data[3] * 1 - 50     #1 degree c/bit offset 50
        Pack5_T = msg.data[4] * 1 - 50     #1 degree c/bit offset 50
        max_temp = max(Pack1_T, Pack2_T, Pack3_T, Pack4_T, Pack5_T)

        if max_temp > critical_temp:
            ALL_GPIO_control(Pump_control_Pin, 1)
        else:
            ALL_GPIO_control(Pump_control_Pin, 0)

        temp_L = max_temp % 256
        temp_H = max_temp / 256

        BMS_data2 = battery_management_system_msg()
        BMS_data2.currenttime = time.time()
        BMS_data2.Pack1_T = Pack1_T
      	BMS_data2.Pack2_T = Pack2_T
      	BMS_data2.Pack3_T = Pack3_T
      	BMS_data2.Pack4_T = Pack4_T
      	BMS_data2.Pack5_T = Pack5_T
        BMS_data2.mode = 2
        BMS_pub.publish(BMS_data2)
        
        return temp_H, temp_L

class IMU:

    @staticmethod
    def get_accelaration(msg, mode):
        if mode == 'Acc':
            const = 100
        elif mode == 'Ang':
            const = 128

        x = (msg.data[3]*256+msg.data[2]) - 32000 
        y = (msg.data[1]*256+msg.data[0]) - 32000 
        z = (msg.data[5]*256+msg.data[4]) - 32000 
        x /= const
        y /= const
        z /= const
            
        IMU_data1 = inertial_measurement_unit_msg()
        if mode == 'Acc':
            IMU_data1.x = x
            IMU_data1.y = y
            IMU_data1.z = z
            IMU_data1.type = 1
        elif mode == 'Ang':
        	IMU_data1.x = x
            IMU_data1.y = y
            IMU_data1.z = z
            IMU_data1.type = 2
      	IMU_data1.currenttime = time.time()
      	IMU_data1.mode = 1
        IMU_pub.publish(IMU_data1)

        return x, y, z

    @staticmethod
    def get_position(msg):
        my = (msg.data[2]*65536+msg.data[1]*256+msg.data[0]) - 8192000
        my /= 32768
        mx = (msg.data[5]*65536+msg.data[4]*256+msg.data[3]) - 8192000
        mx /= 32768
        
        IMU_data2 = inertial_measurement_unit_msg()
        IMU_data2.currenttime = time.time()
        IMU_data2.my = my
        IMU_data2.mx = mx
        IMU_data1.mode = 2
        IMU_pub.publish(IMU_data2)

        return my, mx
   
def Traction_control(speed_f, speed_r):
    Traction_error_new = speed_r - speed_f

    if Traction_error_new > 0:
        P = 3.2
        I = 0
        Traction_error_differnece = Traction_error_new - Traction_error_old
        pedal_adjust_const = Traction_error_new * P + Traction_error_difference * I
    else:
        pedal_adjust_const = 0.0
        
    Traction_error_old = Traction_error_new

#########################################################################
#                                                                       #
#                              Start Here !!!                           #
#                                                                       #
#########################################################################

err_dict = {0: 'BMS Error',
            1: 'Over charging Current',
            2: 'Over Discharging Current',
            3: 'Under Voltage',
            4: 'Under Temperature',
            5: 'Over Voltage',
            6: 'Low SOC',
            7: 'Over Temperature'
            }    

# [imu.data, mode]

if __name__ == "__main__":
    os.system('sudo ip link set can0 type can bitrate 250000')
    os.system('sudo ifconfig can0 txqueuelen 1000')
    os.system('sudo ifconfig can0 up')

    Led_Pin = 12                 #Brake Pin
	Vcu_shutdown_Pin = 32
	WarningLED1_Pin = 29         #Canbus fail
	WarningLED2_Pin = 31         #different accpedal signal
	Brake_sig_Pin = 36           #for BSPD
	Bat_shutdown_Pin = 15        #overheated
	Pump_control_Pin = 12        #turn on pump when over critical temp

    can0 = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')

    strftime_var = f'%Y%m%d_%H:%M:%S{time.localtime()}'
    Datafile_name = f'{time.strftime(strftime_var)}.txt'
    file_path = f'./Data_folder/{Datafile_name}'

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    GPIO.setup(Led_Pin, GPIO.OUT)
    GPIO.setup(Vcu_shutdown_Pin, GPIO.OUT)
    GPIO.setup(WarningLED1_Pin, GPIO.OUT)
    GPIO.setup(WarningLED2_Pin, GPIO.OUT)
    GPIO.setup(Brake_sig_Pin, GPIO.OUT)
    GPIO.setup(Bat_shutdown_Pin, GPIO.OUT)
    GPIO.setup(Pump_control_Pin, GPIO.OUT)

    spdL = 0
    spdH = 0
    fspdL = 0
    fspdH = 0
    tempL = 0
    tempH = 0
    SOC = 0
    max_V = 0
    min_V = 0
    critical_temp = 35 #degC
    tire_radius = 0.23 #m
    rear_car_speed = 0 #m/s
    front_car_speed = 0 #m/s
    pedal_adjust_const = 0.0 #when slip happen, hold back the acc_pedal value

    try:
        while True:
            msg = can0.recv(1.0)
            canid.publish(msg)

            if msg is None:
                ALL_GPIO_control(WarningLED1_Pin, 1)
                print('Timeout , no message.')

            else:
                ALL_GPIO_control(WarningLED1_Pin, 0)

                if msg.arbitration_id == 0x080AD091:
                    print('Receive message from Arduino 1')
                    acc_pedal, brk_pedal, fr_wheel_angspeed, fl_wheel_angspeed, fspdL, fspdH = Arduino.decode_1(msg)
                    print('fspdL : '+ fspdL +'/fspdH : '+ fspdH +'\n')
                    acc_pedal = int(acc_pedal - pedal_adjust_const)

                    msg1 = can.Message(arbitration_id = 0x0008A7D0,
                                       data = [20,acc_pedal,brk_pedal,0,90,0,234,0], extended_id = True)
                    msg2 = can.Message(arbitration_id = 0x0008A8D0,
                                       data = [20,acc_pedal,brk_pedal,0,90,0,234,0], extended_id = True)
                    print('send to mcu')
                    can0.send(msg1)
                    can0.send(msg2)
                    
                elif msg.arbitration_id == 0x080AD092:
                    print('Receive message from Arduino 2')
                    steer_angle, rr_wheel_angspeed, rl_wheel_angspeed = Arduino.decode_2(msg)

                elif msg.arbitration_id == 0x0808D0A7:
                    print('Receive message from MCU 1 State 1')
                    MCU.state1(msg,1)

                elif msg.arbitration_id == 0x0809D0A7:
                    print('Receive message from MCU 1 State 2')
                    spdL, spdH = MCU.state2(msg,1)

                elif msg.arbitration_id == 0x0808D0A8:
                    print('Receive message from MCU 2 State 1')
                    MCU.state1(msg,2)

                elif msg.arbitration_id == 0x0809D0A8:
                    print('Receive message from MCU 2 State 2')
                    MCU.state2(msg,2)

                elif msg.arbitration_id == 0x1808D0F4:
                    print('Receive message from BMS (System)')
                    Max_V, min_V, SOC = BMS.state(msg)

                elif msg.arbitration_id == 0x1809D0F4:
                    print('Receive message from BMS (Battery Thermo)')
                    tempL, tempH = BMS.pack(msg)

                elif msg.arbitration_id == 0x08f02de2:
                    print('Receive message from IMU Acceleration')
                    x, y, z = IMU.get_accelaration(msg, 'Acc')

                elif msg.arbitration_id == 0xcf02ae2:
                    print('Receive message from IMU Angular acceleration')
                    x, y, z = IMU.get_accelaration(msg, 'Ang')

                elif msg.arbitration_id == 0x0cf029e2:
                    print('Receive message from IMU Posture')
                    my, mx = IMU.get_position(msg)

                else:
                    print(msg)

                msgDashboard = can.Message(arbitration_id = 0x1600d027,data = [int(fspdL),int(fspdH),int(tempL),int(tempH),SOC,0,0],extended_id = True)
                can0.send(msgDashboard)

    except Exception as e:
        can0.flush_tx_buffer()
        print(e)