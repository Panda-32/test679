#!/usr/bin/env pybricks-micropython
 
 
from pybricks.ev3devices import Motor, ColorSensor , GyroSensor
from pybricks.parameters import Port,Color, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from time import sleep
 
# Initialize the motors.
left_motor = Motor(Port.A,positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D,positive_direction=Direction.COUNTERCLOCKWISE)
#gyro = GyroSensor(Port.S1) 
 

# Initialize the color sensor.
line_sensor_left = ColorSensor(Port.S4)
color1 = line_sensor_left.reflection()


 
 
# Initialize the drive base.
#robot = DriveBase(left_motor, right_motor, wheel_diameter=5, axle_track=104)
#motor_speedlog =  Datalog('error' , 'intergral' , 'derivative'  , 'turn_rate ' , name = 'motor_speedlog')

#robot.straight(100)


BLACK = 3
WHITE = 50
threshold = 25

DRIVE_SPEED = 50
PROPORTIONAL_GAIN = 0.80
INTERAL_GAIN = 0
DERIVATIVE_GAIN = 0

intergral = 0
derivative = 0
#proportional = 0
last_error = 0
 #following the line endlessly.

f = open("test.txt","w")
#f.write("testing")
t=0
while True:
    color1 = line_sensor_left.reflection()
    error = color1 - threshold
    intergral = intergral + error
    derivative = error - last_error 
    turn_rate = PROPORTIONAL_GAIN * error + INTERAL_GAIN * intergral  + DERIVATIVE_GAIN * derivative
    #turn_rate = turn_rate/100
    
    left_motor.run(DRIVE_SPEED - turn_rate)
    right_motor.run(DRIVE_SPEED + turn_rate)
    last_error = error
    t=t+1

f.close()


