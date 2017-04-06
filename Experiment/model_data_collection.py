import mraa  
import time  
import numpy as np
from numpy import linalg
import math
import scipy as sio
from servo import Servo 
from stepper import Stepper
import linalgfunc
import pdb
import time

global pi
pi = np.pi

global sin
sin = np.sin

global asin
asin = np.arcsin

global cos
cos = np.cos

global atan2
atan2 = np.arctan2

def tand(x):
    tempx = np.multiply(x,pi/180.0)
    return tand(tempx)

def atan2d(x,y):
    temp_theta = atan2(x,y)
    return np.multiply(temp_theta,180.0/pi)

def sind(x):
    tempx = np.multiply(x,pi/180.0)
    return sin(tempx)

def asind(x):
    temp_theta = asin(x)
    return np.multiply(temp_theta,180.0/pi)

def cosd(x):
    tempx = np.multiply(x,pi/180.0)
    return cos(tempx)

def angle_transform(alpha,beta,theta):
	#transform the local coordinates alpha, beta into global coordinates, when
	#the local coordinate is at elevation theta (use -theta for inversing the transform)
    alpha_prime = asind(cosd(alpha)*cosd(beta)*sind(theta) + sind(alpha)*cosd(theta))
    beta_prime = atan2d(cosd(alpha)*sind(beta), cosd(alpha)*cosd(beta)*cosd(theta) - sind(alpha)*sind(theta))
    return (alpha_prime,beta_prime)

def setup():
	global LED
	LED = mraa.Gpio(7)  
	LED.dir(mraa.DIR_OUT) 

	global Light
	Light = mraa.Gpio(8)  
	Light.dir(mraa.DIR_OUT) 

	global sensorPin
	sensorPin = mraa.Aio(5)

	global BaseStepper
	BaseStepper = Stepper(7,6,2)

	global ReceiverStepper
	ReceiverStepper = Stepper(10,11,9)

def initialize():
	global num_iteration
	num_iteration = 300

def getIntensity():
	total_intensity = 0
	for inum in range(1,100):
		v = (sensorPin.read()/1024.0)*7.6
		total_intensity = total_intensity + v

	avg_intensity = total_intensity/inum
	return avg_intensity


def onLED():
	LED.write(1)

def offLED():
	LED.write(0)

def onLights():
	Light.write(0)

def offLights():
	Light.write(1)

#Main function execution starts here
setup()
initialize()

#Variables Initialization

onLights()
b = 30  #length of Stepper scan
l = 2*b+1
theta_step = 1
psi_step = 1
model_data = np.zeros((l,l))
#model_data = np.zeros(l1)
psi_data = np.zeros((l,l))
theta_data = np.zeros((l,l))
#theta_data = np.zeros(l1)
theta = -theta_step*(b) 
ReceiverStepper.rotateMotor(-theta)
time.sleep(1)

psi = -psi_step*(b) 
BaseStepper.rotateMotor(psi)
time.sleep(1)


rotation = 1
k = 0
i=0
for i in range(0,l-1):
	for j in range(0,l-1):
		theta_data[i,k] = theta
		psi_data[i,k] = psi
		print [theta,psi]
		time.sleep(0.500)
		model_data[i,j] = getIntensity()
		print model_data[i,j]
		ReceiverStepper.rotateMotor(-rotation*theta_step)  #Note that the command is incremental
		theta = theta + rotation*theta_step
		k = k + rotation
	time.sleep(0.500)
	rotation = -rotation
	#Compensating the last step
	ReceiverStepper.rotateMotor(-rotation*theta_step)  #Note that the command is incremental
	theta = theta + rotation*theta_step
	BaseStepper.rotateMotor(psi_step)
	psi = psi + psi_step



np.savez('model_data_step3.npz', model_data= model_data, psi_data = psi_data,theta_data = theta_data)
offLights()
myStepper.rotateMotor(-psi)
myServo.write(0)

offLights()
np.savez('servo_data.npz', theta_data = theta_data,model_data = model_data)