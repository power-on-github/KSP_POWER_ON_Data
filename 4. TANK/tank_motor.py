# from serial import Serial
# 
# ser = Serial('dev/ttyACM0',115200)
# 
# #ser.write(bytes(str.encode()))
# while True:
#     if ser.readable():
#         res = ser.readline()
#         print(res.decode()[:len(res) - 1])

import serial
# import wiringpi
from time import sleep

ser = serial.Serial("/dev/serial0", 115200)  # Open port with baud rate

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0
CH2 = 1

OUTPUT = 1
INPUT = 0

HIGH = 1
LOW = 0

# PWM pin
ENA = 25

# GPIO pin
IN1 = 24
IN2 = 23

# DC pin setting func
"""
def setPinConfig(EN, INA, INB):
    wiringpi.pinMode(EN, OUTPUT)
    wiringpi.pinMode(INA, OUTPUT)
    wiringpi.pinMode(INB, OUTPUT)
    wiringpi.softPwmCreate(EN, 0, 255)


# DC motor control
def setMotorControl(PWM, INA, INB, speed, stat):
    #    motor speed control pwm
    wiringpi.softPwmWrite(PWM, speed)

    #    forward
    if stat == FORWARD:
        wiringpi.digitalWrite(INA, HIGH)
        wiringpi.digitalWrite(INB, LOW)
    #    backward
    elif stat == BACKWARD:
        wiringpi.digitalWrite(INA, LOW)
        wiringpi.digitalWrite(INB, HIGH)
    elif stat == STOP:
        wiringpi.digitalWrite(INA, LOW)
        wiringpi.digitalWrite(INB, LOW)


# DC wrapping functions for convenience
def setMotor(ch, speed, stat):
    if ch == CH1:
        setMotorControl(ENA, IN1, IN2, speed, stat)
    else:
        print("wrong channel")
        setMotorControl(ENB, IN3, IN4, speed, stat)


# DC setting gpio library
wiringpi.wiringPiSetup()

# DC setting motor pins
setPinConfig(ENA, IN1, IN2)

"""
# main
while True:
    received_data = ser.read(4)  # read serial port
    sleep(0.08)
    if (received_data[0] == 10) and (received_data[1] == 10):
        print('detect:', received_data[2], received_data[3])

    data_left = ser.inWaiting()  # check for remaining byte
    received_data += ser.read(data_left)

    # print(res.decode()[:len(res)]) #print received data
    #print('data', received_data)
"""
    if res[0] > 127:
        print(res[0])
        print('big')
        print("start motor")
        setMotor(CH1, 255, FORWARD)
        wiringpi.delay(1000)
        setMotor(CH1, 0, STOP)
    else:
        print('small')
        print("start motor")
        setMotor(CH1, 255, BACKWARD)
        wiringpi.delay(1000)
        setMotor(CH1, 0, STOP)
"""
# ser.write(received_data)  # transmit data serially
