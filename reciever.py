#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32

import os,sys
import time
import math

import numpy as np
import can
import RPi.GPIO as GPIO
import threading

from GPS_stuff import GPS_serial_setup, GPS_read
from IP_stuff import Send_IP_info


rospy.init_node('reciever_node')
canid = rospy.Publisher("id", Float32, queue_size=10)
arduino1_pub = rospy.Publisher("arduino1", Float32, queue_size=10)
arduino2_pub = rospy.Publisher("arduino2", Float32, queue_size=10)
MCU_pub = rospy.Publisher("motor_control_unit", Float32, queue_size=10)
# 兩個MCU function的作用？
BMS_pub = rospy.Publisher("battery_management_system", Float32, queue_size=10)
IMU_pub = rospy.Publisher("inertial_measurement_unit", Float32, queue_size=10)


Led_Pin = 12                 #Brake Pin
Vcu_shutdown_Pin = 32
WarningLED1_Pin = 29         #Canbus fail
WarningLED2_Pin = 31         #different accpedal signal
Brake_sig_Pin = 36           #for BSPD
Bat_shutdown_Pin = 15        #overheated
Pump_control_Pin = 12        #turn on pump when over critical temp

tempL = 0
tempH = 0
SOC = 0
max_V = 0
min_V = 0
critical_temp = 35 #degC
tire_radius = 0.225 #m
rear_car_speed = 0 #m/s
front_car_speed = 0 #m/s
pedal_adjust_const = 0.0 #when slip happen, hold back the acc_pedal value

inittime = time.time()  # initial time stamp


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
        acc_pedal1 = msg.data[0]
        acc_pedal2 = msg.data[1]
        brake_pedal1 = msg.data[2]  # 0.39%/bit
        brake_pedal2 = msg.data[3]
        fl_wheel_angspeed = (msg.data[4]*256 + msg.data[5]*1) * 0.002   # 0.002rad/s/bit
        fr_wheel_angspeed = (msg.data[6]*256 + msg.data[7]*1) * 0.002   # 0.002rad/s/bit

        front_car_speed = (fl_wheel_angspeed+fr_wheel_angspeed)*tire_radius*math.pi*3.6*2
        front_car_speedL = front_car_speed % 256
        front_car_speedH = int(front_car_speed / 256)
        
        print(f'acc 1 : {acc_pedal1*0.39}%\nacc 2 : {acc_pedal2*0.39}%')
        print(f'fl_wheel_angspeed : {fl_wheel_angspeed}\nfront_car_speedL : {front_car_speedL}\nfront_car_speedH : {front_car_speedH}')

        # with open('Data/Arduino1_decode.csv', 'a') as f:
        	# currenttime = time.time() #- inittime
            # f.write(f'{currenttime},{acc_pedal1*0.39},{acc_pedal2*0.39},{brake_pedal1*0.39},{brake_pedal2*0.39},{fl_wheel_angspeed},{fr_wheel_angspeed}\n')

        currenttime = time.time()
        arduino1_data = f'{currenttime},{acc_pedal1*0.39},{acc_pedal2*0.39},{brake_pedal1*0.39},{brake_pedal2*0.39},{fl_wheel_angspeed},{fr_wheel_angspeed}\n'
        # 之後改成list
        arduino1_pub.publish(arduino1_data)



        if brake_pedal1 > 0 or brake_pedal2 > 0:    # 255 * 0.75 = 191.25
            ALL_GPIO_control(Led_Pin, 1)
        else:
            ALL_GPIO_control(Led_Pin, 0)

        return acc_pedal1, brake_pedal1, fr_wheel_angspeed, fl_wheel_angspeed, front_car_speedL, front_car_speedH

    @staticmethod
    def decode_2(msg):
        steer_angle = msg.data[0]*100/255   #reck:0–>left 100–>right
        rl_wheel_angspeed = (msg.data[2]*256+msg.data[3]*1) * 0.002  # 0.002rad/s/bit
        rr_wheel_angspeed = (msg.data[4]*256+msg.data[5]*1) * 0.002  # 0.002rad/s/bit
        rear_avg_wheel_angspeed = (rl_wheel_angspeed + rr_wheel_angspeed)/2
        rear_car_speed = rear_avg_wheel_angspeed * tire_radius #m/s

        #with open('Data/Arduino2_decode.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{steer_angle},{rr_wheel_angspeed},{rl_wheel_angspeed}\n')

        currenttime = time.time()
        arduino2_data = f'{currenttime},{steer_angle},{rr_wheel_angspeed},{rl_wheel_angspeed}\n'
        # 之後改成list
        arduino2_pub.publish(arduino2_data)

        return steer_angle, rr_wheel_angspeed, rl_wheel_angspeed

class MCU:

    @staticmethod
    def state1(msg,n):

        motor_T = msg.data[6]
        controller_T = msg.data[7]

        # with open('Data/MCU_state1.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{motor_T},{controller_T},{n}\n') 

        currenttime = time.time()
        MCU_data1 = f'{currenttime},{motor_T},{controller_T},{n}\n'
        # 之後改成list
        MCU_pub.publish(MCU_data1)

    @staticmethod
    def state2(msg,n):
        battery_V = (msg.data[0]*256+msg.data[1]*1) / 128  #1/128 V/bit
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

        # with open('Data/MCU_state2.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{battery_V},{battery_I},{motor_I},{motor_speed},{car_speed},{n}\n')

        currenttime = time.time()
        MCU_data2 = f'{currenttime},{battery_V},{battery_I},{motor_I},{motor_speed},{car_speed},{n}\n'
        # 之後改成list
        MCU_pub.publish(MCU_data2)
        
        return car_speedL, car_speedH

class BMS:

    @staticmethod
    def state(msg):
        Pack_voltage = (msg.data[0]*256+msg.data[1]) * 0.01 # V
        SOC = msg.data[2]
        Max_CellV = (msg.data[3]*256+msg.data[4]) * 0.1 #mV
        Min_CellV = (msg.data[5]*256+msg.data[6]) * 0.1 #mV
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

        # with open('Data/BMS_State.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{Pack_voltage},{SOC}\n')

     	currenttime = time.time()
        BMS_data1 = f'{currenttime},{Pack_voltage},{SOC}\n'
        # 之後改成list
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

        # with open('Data/BMS_Pack.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{Pack1_T},{Pack2_T},{Pack3_T},{Pack4_T},{Pack5_T}\n')

        currenttime = time.time()
        BMS_data2 = f'{currenttime},{Pack1_T},{Pack2_T},{Pack3_T},{Pack4_T},{Pack5_T}\n'
        # 之後改成list
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
            
        if mode == 'Acc':
            # with open('Data/IMU_Acceleration.csv', 'a') as f:
            #     currenttime = time.time() #- inittime
            #     f.write(f'{currenttime},{x},{y},{z}\n')
            data = f'{currenttime},{x},{y},{z}\n'

        elif mode == 'Ang':
            # with open('Data/IMU_Angular_Speed.csv', 'a') as f:
            #     currenttime = time.time() #- inittime
            #     f.write(f'{currenttime},{x},{y},{z}\n')
            data = f'{currenttime},{x},{y},{z}\n'

      	currenttime = time.time()
        IMU_data1 = data
        # 之後改成list
        IMU_pub.publish(IMU_data1)

        return x, y, z

    @staticmethod
    def get_position(msg):
        my = (msg.data[2]*65536+msg.data[1]*256+msg.data[0]) - 8192000
        my /= 32768
        mx = (msg.data[5]*65536+msg.data[4]*256+msg.data[3]) - 8192000
        mx /= 32768
        
        # with open('Data/IMU_Attitude.csv', 'a') as f:
        #     currenttime = time.time() #- inittime
        #     f.write(f'{currenttime},{my},{mx}\n')

        currenttime = time.time()
        IMU_data2 = f'{currenttime},{my},{mx}\n'
        # 之後改成list
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

def run(var):
    var=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
    

    os.system('sudo ip link set can0 type can bitrate 250000')
    os.system('sudo ifconfig can0 txqueuelen 1000')
    os.system('sudo ifconfig can0 up')


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
                    print(f'fspdL : {fspdL}/fspdH : {fspdH}\n')
                    with threading.Lock():
                        var[0] = acc_pedal / 2
                        var[1] = brk_pedal / 2
                        var[3] = fr_wheel_angspeed
                        var[4] = fl_wheel_angspeed
                    acc_pedal = int(acc_pedal - pedal_adjust_const)
                    msg1 = can.Message(arbitration_id = 0x0008A7D0,
                                       data = [20,acc_pedal,brk_pedal,0,90,0,234,0],
                                       extended_id = True)
                    msg2 = can.Message(arbitration_id = 0x0008A8D0,
                                       data = [20,acc_pedal,brk_pedal,0,90,0,234,0],
                                       extended_id = True)
                    print('send to mcu')
                    can0.send(msg1)
                    can0.send(msg2)
                    
                elif msg.arbitration_id == 0x080AD092:
                    print('Receive message from Arduino 2')
                    steer_angle, rr_wheel_angspeed, rl_wheel_angspeed = Arduino.decode_2(msg)
                    with threading.Lock():
                        var[2] = steer_angle
                        var[5] = rr_wheel_angspeed
                        var[6] = rl_wheel_angspeed

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
                    with threading.Lock():
                        var[10] = x
                        var[11] = y
                        var[12] = z

                elif msg.arbitration_id == 0xcf02ae2:
                    print('Receive message from IMU Angular acceleration')
                    x, y, z = IMU.get_accelaration(msg, 'Ang')
                    with threading.Lock():
                        var[13] = x
                        var[14] = y
                        var[15] = z

                elif msg.arbitration_id == 0x0cf029e2:
                    print('Receive message from IMU Posture')
                    my, mx = IMU.get_position(msg)
                    with threading.Lock():
                        var[17] = y
                        var[18] = z

                else:
                    print(msg)

                msgDashboard = can.Message(arbitration_id = 0x1600d027,data = [int(fspdL),int(fspdH),int(tempL),int(tempH),SOC,0,0],extended_id = True)
                can0.send(msgDashboard)
    except Exception as e:
        can0.flush_tx_buffer()
        print(e)


if __name__ == "__main__":
    var={0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}
    run(var)