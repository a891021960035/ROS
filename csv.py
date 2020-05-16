#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32

ID = [0]

def get_id(data):
	ID[0] = data.data

def csv_arduino1(data):
	string = data.data
	with open('Data/Arduino1_decode.csv', 'a') as f:
        f.write(string)

def csv_arduino2(data):
	string = data.data
	with open('Data/Arduino2_decode.csv', 'a') as f:
        f.write(string)

def csv_motor_control_unit_state1(data):
	string = data.data
	with open('Data/MCU_state1.csv', 'a') as f:
        f.write(data)

def csv_motor_control_unit_state2(data):
	string = data.data
	with open('Data/MCU_state2.csv', 'a') as f:
        f.write(data)

def csv_battery_management_system_state(data):
	string = data.data
	with open('Data/BMS_State.csv', 'a') as f:
		f.write(data)

def csv_battery_management_system_pack(data):
	string = data.data
	with open('Data/BMS_Pack.csv', 'a') as f
		f.write(data)

def csv_inertial_measurement_unit_accelaration(data):
	string = data.data
	with open('Data/IMU_Angular_Speed.csv', 'a') as f:
		f.write(data)

def csv_inertial_measurement_unit_position(data):
	string = data.data
	with open('Data/IMU_Attitude.csv', 'a') as f:
		f.write(data)

def listener():
    rospy.init_node('v_control', anonymous=True)
    rospy.Subscriber("id", Float32, get_id'''怎麼單純使用變數''')
    rospy.Subscriber("arduino1", Float32, csv_arduino1)
    rospy.Subscriber("arduino2", Float32, csv_arduino2)

    if ID[0] == 0x0808D0A7 or 0x0808D0A8:
    	rospy.Subscriber("motor_control_unit", Float32, csv_motor_control_unit_state1)
    elif ID[0] == 0x0809D0A7 or 0x0809D0A8:
    	rospy.Subscriber("motor_control_unit", Float32, csv_motor_control_unit_state2)

    if ID[0] == 0x1808D0F4:
    	rospy.Subscriber("battery_management_system", Float32, csv_battery_management_system_state)
	elif ID[0] == 0x1809D0F4:
		rospy.Subscriber("battery_management_system", Float32, csv_battery_management_system_pack)

	if ID[0] == 0x08f02de2 or 0xcf02ae2:
    	rospy.Subscriber("inertial_measurement_unit", Float32, csv_inertial_measurement_unit_accelaration)
	elif ID[0] == 0x0cf029e2:
		rospy.Subscriber("inertial_measurement_unit", Float32, csv_inertial_measurement_unit_position)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
	with open('Data/Arduino1_decode.csv', 'w') as f:
        f.write('Arduino1_decode Input:\r\n')
        f.write('time,throttle1 (%),throttle2 (%),brake1 (%),brake1 (%),brake2 (%),fl wheelspeed (rad/s),f2 wheelspeed (rad/s)\n')   
    with open('Data/Arduino2_decode.csv', 'w') as f:
        f.write('Arduino2_decode Input:\r\n')
        f.write('time,steer angle (%),rl wheelspeed (rad/s),rf wheelspeed (rad/s)\n')
    with open('Data/MCU_state1.csv', 'w') as f:
        f.write('MCU_state1 Input:\r\n')
        f.write('time,MMotor Temp (deg C),Controller Temp (deg C),MCU number\n')
    with open('Data/MCU_state2.csv', 'w') as f:
        f.write('MCU_state2 Input:\r\n')
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