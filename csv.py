#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32
from ntu_racing.msg import arduino_decode1_msg
from ntu_racing.msg import arduino_decode2_msg
from ntu_racing.msg import motor_control_unit_msg
from ntu_racing.msg import battery_management_system_msg
from ntu_racing.msg import inertial_measurement_unit_msg


def csv_arduino1(data):
	Data = data.data
	currenttime = str(Data.currenttime)
	acc_pedal1 = str(Data.acc_pedal1)
	acc_pedal2 = str(Data.acc_pedal2)
	brake_pedal1 = str(Data.brake_pedal1)
	brake_pedal2 = str(Data.brake_pedal2)
	fl_wheel_angspeed = str(Data.fl_wheel_angspeed)
	fr_wheel_angspeed = str(Data.fr_wheel_angspeed)

	with open('Data/Arduino1_decode.csv', 'a') as f:
        f.write(f'{currenttime},{acc_pedal1},{acc_pedal2},{brake_pedal1},{brake_pedal2},{fl_wheel_angspeed},{fr_wheel_angspeed}\n')

def csv_arduino2(data):
	Data = data.data
	currenttime = str(Data.currenttime)
	steer_angle = str(Data.steer_angle)
	rr_wheel_angspeed = str(Data.rr_wheel_angspeed)
	rl_wheel_angspeed = str(Data.rl_wheel_angspeed)

	with open('Data/Arduino2_decode.csv', 'a') as f:
        f.write(f'{currenttime},{steer_angle},{rr_wheel_angspeed},{rl_wheel_angspeed}\n')

def csv_motor_control_unit(data):
	Data = data.data
	currenttime = str(Data.currenttime)

	motor_T = str(Data.motor_T)
	controller_T = str(Data.controller_T)

	battery_V = str(Data.battery_V)
	battery_I = str(Data.battery_I)
	motor_I = str(Data.motor_I)
	motor_speed = str(Data.motor_speed)
	car_speed = str(Data.car_speed)

	if Data.state == 1:
		if Data.num == 1:
			with open('Data/MCU1_state1.csv', 'a') as f:
	        	f.write(f'{currenttime},{motor_T},{controller_T},{n}\n') 
	    elif Data.num == 2:
	    	with open('Data/MCU2_state1.csv', 'a') as f:
	        	f.write(f'{currenttime},{motor_T},{controller_T},{n}\n')
	elif Data.state == 2:
		if Data.num == 1:
			with open('Data/MCU1_state2.csv', 'a') as f:
	        	f.write(f'{currenttime},{battery_V},{battery_I},{motor_I},{motor_speed},{car_speed}\n') 
	    elif Data.num == 2:
	    	with open('Data/MCU2_state2.csv', 'a') as f:
	        	f.write(f'{currenttime},{battery_V},{battery_I},{motor_I},{motor_speed},{car_speed}\n') 

def csv_battery_management(data):
	Data = data.data
	currenttime = str(Data.currenttime)

	Pack_voltage = str(Data.Pack_voltage)
	SOC = str(Data.SOC)

	Pack1_T = str(Data.Pack1_T)
	Pack2_T = str(Data.Pack2_T)
	Pack3_T = str(Data.Pack3_T)
	Pack4_T = str(Data.Pack4_T)
	Pack5_T = str(Data.Pack5_T)

	if Data.mode == 1:
		with open('Data/BMS_State.csv', 'a') as f:
	        f.write(f'{currenttime},{Pack_voltage},{SOC}\n')
    elif Data.mode == 2:
    	with open('Data/BMS_Pack.csv', 'a') as f
        	f.write(f'{currenttime},{Pack1_T},{Pack2_T},{Pack3_T},{Pack4_T},{Pack5_T}\n')

def csv_inertial_measurement_unit_accelaration(data):
	Data = data.data
	currenttime = str(Data.currenttime)

	x = str(Data.x)
	y = str(Data.y)
	z = str(Data.z)

	my = str(Data.my)
	mx = str(Data.mx)

	if Data.mode == 1:
		if Data.type == 1:
			with open('Data/IMU_Acceleration.csv', 'a') as f:
                f.write(f'{currenttime},{x},{y},{z}\n')            
        elif Data.type == 2:
			with open('Data/IMU_Angular_Speed.csv', 'a') as f:
                f.write(f'{currenttime},{x},{y},{z}\n')
    elif Data.mode == 2:
		with open('Data/IMU_Attitude.csv', 'a') as f:
            f.write(f'{currenttime},{my},{mx}\n')

def listener():
    rospy.init_node('v_control', anonymous=True)
    rospy.Subscriber("arduino1", arduino_decode1_msg, csv_arduino1)
    rospy.Subscriber("arduino2", arduino_decode2_msg, csv_arduino2)
    rospy.Subscriber("motor_control_unit", motor_control_unit_msg, csv_motor_control_unit)    
	rospy.Subscriber("battery_management_system", battery_management_system_msg, csv_battery_management_system)
	rospy.Subscriber("inertial_measurement_unit", inertial_measurement_unit_msg, csv_inertial_measurement_unit)
	# spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
	with open('Data/Arduino1_decode.csv', 'w') as f:
        f.write('Arduino1_decode Input:\r\n')
        f.write('time,throttle1 (%),throttle2 (%),brake1 (%),brake1 (%),brake2 (%),fl wheelspeed (rad/s),f2 wheelspeed (rad/s)\n')   
    with open('Data/Arduino2_decode.csv', 'w') as f:
        f.write('Arduino2_decode Input:\r\n')
        f.write('time,steer angle (%),rl wheelspeed (rad/s),rf wheelspeed (rad/s)\n')
    with open('Data/MCU1_state1.csv', 'w') as f:
        f.write('MCU1_state1 Input:\r\n')
        f.write('time,Motor Temp (deg C),Controller Temp (deg C),MCU number\n')
    with open('Data/MCU2_state1.csv', 'w') as f:
        f.write('MCU2_state1 Input:\r\n')
        f.write('time,Motor Temp (deg C),Controller Temp (deg C),MCU number\n')
    with open('Data/MCU1_state2.csv', 'w') as f:
        f.write('MCU1_state2 Input:\r\n')
        f.write('time,Battery Voltage (V),Battery Current (A),Motor Current (A),Motor Speed (rpm),Car Speed (m/s),MCU number\n')
    with open('Data/MCU2_state2.csv', 'w') as f:
        f.write('MCU2_state2 Input:\r\n')
        f.write('time,Battery Voltage (V),Battery Current (A),Motor Current (A),Motor Speed (rpm),Car Speed (m/s),MCU number\n')
    with open('Data/BMS_State.csv', 'w') as f:
        f.write('BMS_State Input:\r\n')
        f.write('time,Pack Voltage (V),SOC (%)\n')
    with open('Data/BMS_Pack.csv', 'w') as f:
        f.write('BMS_Pack Input:\r\n')
        f.write('time,Pack1 temp (deg C),Pack2 temp (deg C),Pack3 temp (deg C),Pack4 temp (deg C),Pack5 temp (deg C)\n')
    with open('Data/IMU_Acceleration.csv', 'w') as f:
        f.write('IMU_Acceleration Input:\r\n')
        f.write('time,Acceleration x (m/s^2),Acceleration y (m/s^2),Acceleration z (m/s^2)\n')
    with open('Data/IMU_Angular_Speed.csv', 'w') as f:
        f.write('IMU_Angular_Speed Input:\r\n')
        f.write('time,Angular velocity x (rad/s),Angular velocity y (rad/s),Angular velocity z (rad/s)\n')
    with open('Data/IMU_Attitude.csv', 'w') as f:
        f.write('IMU_Attitude Input:\r\n')
        f.write('time,Roll my (deg),Pitch mx (deg)\n')
    listener()