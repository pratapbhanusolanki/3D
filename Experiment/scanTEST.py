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

	global sensorPin
	sensorPin = mraa.Aio(2)

	global myStepper
	myStepper = Stepper()

	global myServo
	myServo = Servo()
	myServo.attach(9)
	myServo.write(30)

def initialize():
	global num_iteration
	num_iteration = 300

	global A 
	A = np.identity(3)

	global I
	I = np.identity(3) 

	global B
	B = np.matrix([[0,0],[1,0],[0,1]])

	global Q
	Q = np.matrix([[1,0,0],[0,10,0],[0,0,10]])
	Q = 10*Q

	global R
	R = np.identity(3)

	global P_current
	P_current = np.identity(3)

	global scan_parameters_all
	scan_parameters_all = np.zeros((num_iteration,3))

	global x_hatf_all
	x_hatf_all = np.zeros((num_iteration,3))

	global x_hat_all
	x_hat_all = np.zeros((num_iteration,3))

	global y_hat_all
	y_hat_all = np.zeros((num_iteration,3))

	global y_all
	y_all = np.zeros((num_iteration,3))

	global Pf_all
	Pf_all = np.zeros((num_iteration,3,3))

	global P_all
	P_all = np.zeros((num_iteration,3,3))

	global C_all
	C_all = np.zeros((num_iteration,3,3))

	global K_all
	K_all = np.zeros((num_iteration,3,3))


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

#Main function execution starts here
setup()
initialize()



#Variables Initialization
diff_sum = 0
x_hat = np.zeros((3,num_iteration))
x_hat[:,0] = [3,0,0]

angle_bias = np.zeros(num_iteration) 
phi = 90
scan_radius = 10#10
u2_previous = -1.0
u3_previous = -2.0
normal_u2 = 0
normal_u3 = 0
u2 = np.array([[normal_u2], [u2_previous]])
u3 = np.array([[normal_u3], [u3_previous]])
previous_difference = 0
previous_measurement = 2
previous_previous_measurement = 2
psi = np.zeros(num_iteration)
theta = np.zeros(num_iteration)
scan_psi = np.zeros(num_iteration)
scan_theta = np.zeros(num_iteration)
theta[0] = 0

for i in range(1,num_iteration):
	angle_bias[i] = angle_bias[i-1] + phi
	bias = angle_bias[i]
	print bias
	previous_alpha_bias = scan_radius*sind(bias-phi)
	previous_beta_bias = scan_radius*cosd(bias-phi)
	alpha_bias = scan_radius*sind(bias)
	beta_bias = scan_radius*cosd(bias)
    
	psi[i] = psi[i-1] 
	theta[i] = theta[i-1]
	# x_hat[:,i] = x_hat[:,i] + [0,normal_u2,normal_u3]
    
	theta_offset_temp,psi_offset = angle_transform(alpha_bias, beta_bias, theta[i])
	theta_offset = theta_offset_temp-theta[i]
   
	scan_psi[i] = psi[i] + psi_offset
	scan_theta[i] = theta[i] + theta_offset

	Motor_command_stepper = scan_psi[i] - scan_psi[i-1]
	Motor_command_servo = scan_theta[i]     #For servo it is the absolute value, which matters, not the difference with previous one
	print Motor_command_servo
	print Motor_command_stepper
	myServo.write(Motor_command_servo)
	myStepper.rotateMotor(Motor_command_stepper)
	raw_input('input something!: ')