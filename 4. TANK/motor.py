import wiringpi

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0
CH2 = 1

OUTPUT = 1
INPUT = 0

HIGH = 1
LOW = 0

#PWM pin
ENA = 25

#GPIO pin
IN1 = 24
IN2 = 23

#DC pin setting func
def setPinConfig(EN, INA, INB):
    wiringpi.pinMode(EN, OUTPUT)
    wiringpi.pinMode(INA, OUTPUT)
    wiringpi.pinMode(INB, OUTPUT)
    wiringpi.softPwmCreate(EN, 0, 255)

#DC motor control
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

#DC wrapping functions for convenience
def setMotor(ch, speed, stat):
    if ch == CH1:
        setMotorControl(ENA, IN1, IN2, speed, stat)
    else:
        print("wrong channel")
        setMotorControl(ENB, IN3, IN4, speed, stat)

#DC setting gpio library
wiringpi.wiringPiSetup()

#DC setting motor pins
setPinConfig(ENA, IN1, IN2)

print("start motor")
setMotor(CH1, 1500, FORWARD)
