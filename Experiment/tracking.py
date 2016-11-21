import mraa  
import time  
import numpy as np
import math
import scipy as sio
from Servo import Servo
from Stepper import Stepper
  

def setup():
	global LED
	LED = mraa.Gpio(7)  
	LED.dir(mraa.DIR_OUT) 

	global sensorPin
	sensorPin = mraa.Aio(2)

	global myStepper
	myStepper = Stepper

	global myServo
	myServo = Servo
	myServo.attach(9)

def initialize():
	global A 
	A = np.identity(3)

	global I
	I = np.identity(3) 

	global B
	I = np.matrix([[0],[1],[1]])

	global Q
	Q = np.matrix([[1,0,0],[0,10,0],[0,0,10]])
	Q = 0.01*Q

	global R
	R = np.matrix([[1,0],[0,1800]])
	R = 2*R

	global P
	P = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
	Q = 100*P

def getIntensity():
	total_intensity = 0
	for inum in range(1,100):
		v = (sensorPin.read()/1024.0)*7.6
		total_intensity = total_intensity + v

	avg_intensity = total_intensity/inum

def gaussianValue(x)
	a1 = 0.6922/1.0359
   	b1 = 7.752
   	c1 = 148.8
   	a2 = 0.346/1.0359
   	b2 =-13.57
   	c2 = 325.8

   	y = a1*math.exp(-((x-b1)/c1)**2) + a2*math.exp(-((x-b2)/c2)**2)

def gaussianDerivative(x)
	a1 = 0.6922/1.0359
   	b1 = 7.752
   	c1 = 148.8
   	a2 = 0.346/1.0359
   	b2 =-13.57
   	c2 = 325.8
   	y = -2*a1*((x-b1)/c1**2)*math.exp(-((x-b1)/c1)**2) -2*a2*((x-b2)/c2**2)*math.exp(-((x-b2)/c2)**2)


def onLED()
	LED.write(1)

def offLED()
	LED.write(0)


#Main function execution starts here
setup()
initialize()