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

	global sensorPin
	sensorPin = mraa.Aio(5)

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
	Q = np.matrix([[1,0,0],[0,20,0],[0,0,20]])
	Q = 100*Q

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

	global u_all
	u_all = np.zeros((num_iteration,3))


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
scan_radius = 5
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
timer = np.zeros(num_iteration)
theta = np.zeros(num_iteration)
scan_psi = np.zeros(num_iteration)
scan_theta = np.zeros(num_iteration)
theta[0] = 15

start = time.time()
for i in range(1,num_iteration):
	print i
	x_hat_k = x_hat[:,i-1] + [0,normal_u2,normal_u3]
	x_hatf_all[i,:] = x_hat_k

	angle_bias[i] = angle_bias[i-1] + phi
	bias = angle_bias[i]
	previous_alpha_bias = scan_radius*sind(bias-phi)
	previous_beta_bias = scan_radius*cosd(bias-phi)
	alpha_bias = scan_radius*sind(bias)
	beta_bias = scan_radius*cosd(bias)
    
	alpha_diff = alpha_bias - previous_alpha_bias
	beta_diff = beta_bias - previous_beta_bias
    
	previous_u = np.array([u2,u3])
	#print previous_u
	scan_parameters = [scan_radius, bias, phi]
	#print scan_parameters
    
	scan_parameters_all[i,:] = scan_parameters
	C = linalgfunc.get_C_matrix(x_hat_k,previous_u,scan_parameters)
	C_all[i,:,:] = C
	#print C

	P_current = A*P_current*A + Q
	Pf_all[i,:,:] = P_current
	#print P_current
    #Output Calculation
	measurement = getIntensity()
	y = np.array([[measurement],[previous_measurement],[previous_previous_measurement]])
	y_all[i,:] = np.transpose(y)
	#print y

	y_hat = linalgfunc.get_output_array(x_hat_k, previous_u,scan_parameters)
	y_hat_all[i,:] = np.transpose(y_hat)
	y_hat 
	#print y_hat 
	previous_previous_measurement = previous_measurement
	previous_measurement = measurement
	
	#Filtering    
	K = P_current*np.transpose(C)*linalg.inv(C*P_current*np.transpose(C) + R)
	K_all[i,:,:] = K

	x_hat[:,i] = np.array(np.mat(x_hat_k).T+K*(y-y_hat)).T                        
	P_current = (np.identity(3) - K*C)*P_current
	P_all[i,:,:] = P_current

	difference = abs(y[0]-y_hat[0])
	diff_sum = diff_sum + difference
	
	if x_hat[0,i] < 0:
		x_hat[0,i] = 0
	x_hat_all[i,:] =x_hat[:,i]
    
	# if(difference + previous_difference < 2):
	# 	G = 0.2
	# 	G2 = 0.2
	# else:
	# 	G = 0.0
	# 	G2 = 0
	# G = 0
	# G2 = 0
	G=0.2

	previous_difference = difference
	normal_u2 = -G*x_hat[1,i]
	normal_u3 = -G*x_hat[2,i]
	u_all[i,:] = [0,normal_u2,normal_u3]
	print normal_u2 
	print normal_u3
	u2 = np.array([[normal_u2], [u2_previous]])
	u3 = np.array([[normal_u3], [u3_previous]])
	u2_previous = normal_u2
	u3_previous = normal_u3
	# normal_u3 = 0
	# normal_u2 = 0

	dummy_u3,u2_k = angle_transform(normal_u3, normal_u2, theta[i-1])
	u3_k = dummy_u3 - theta[i-1]
	psi[i] = psi[i-1] + u2_k
	theta[i] = theta[i-1] + u3_k
	# x_hat[:,i] = x_hat[:,i] + [0,normal_u2,normal_u3]
    
	theta_offset_temp,psi_offset = angle_transform(alpha_bias, beta_bias, theta[i])
	theta_offset = theta_offset_temp-theta[i]
   
	scan_psi[i] = psi[i] + psi_offset
	scan_theta[i] = theta[i] + theta_offset

    # u2 = np.array([[normal_u2], [u2_previous]])
	# u3 = np.array([[normal_u3], [u3_previous]])

	# u2_previous = normal_u2
	# u3_previous = normal_u3

	Motor_command_stepper = scan_psi[i] - scan_psi[i-1]
	Motor_command_servo = scan_theta[i]     #For servo it is the absolute value, which matters, not the difference with previous one

	myServo.write(Motor_command_servo)
	myStepper.rotateMotor(Motor_command_stepper)
	toc = time.time()
	timer[i] = toc-start

np.savez('data.npz', scan_parameters=scan_parameters, \
	x_hatf_all=x_hatf_all, x_hat=x_hat, Pf_all=Pf_all,\
	C_all=C_all, x_hat_all=x_hat_all, y_hat_all=y_hat_all,\
	y_all=y_all, P_all=P_all, K_all=K_all, timer=timer, u_all=u_all)