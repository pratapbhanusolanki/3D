import mraa  
import time  
import numpy as np
from numpy import linalg
import math
import scipy as sio
from servo import Servo 
from stepper import Stepper
  
pi = math.pi
sin = math.sin
cos = math.cos
atan2 = math.atan2

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
	myServo.write(45)

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

	global P_current
	P_current = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
	Q = 100*P

	global num_iteration
	num_iteration = 1
	x_hat = np.zeros(3,num_iteration)
	x_hat[:,0] = [3,0,0]; 

	global angle_bias
	angle_bias = np.zeros(1,num_iteration)

	global phi 
	phi = 20

	global scan_radius 
	scan_radius = 10

def getIntensity():
	total_intensity = 0
	for inum in range(1,100):
		v = (sensorPin.read()/1024.0)*7.6
		total_intensity = total_intensity + v

	avg_intensity = total_intensity/inum

def gaussianValue(x):
	a1 = 0.6922/1.0359
   	b1 = 7.752
   	c1 = 148.8
   	a2 = 0.346/1.0359
   	b2 =-13.57
   	c2 = 325.8

   	y = a1*math.exp(-((x-b1)/c1)**2) + a2*math.exp(-((x-b2)/c2)**2)

def gaussianDerivative(x):
	a1 = 0.6922/1.0359
   	b1 = 7.752
   	c1 = 148.8
   	a2 = 0.346/1.0359
   	b2 =-13.57
   	c2 = 325.8
   	y = -2*a1*((x-b1)/c1**2)*math.exp(-((x-b1)/c1)**2) -2*a2*((x-b2)/c2**2)*math.exp(-((x-b2)/c2)**2)


def onLED():
	LED.write(1)

def offLED():
	LED.write(0)

def tand(x):
    return tan(x*pi/180.0)

def sind(x):
    return sin(x*pi/180.0)

def cosd(x):
    return cos(x*pi/180.0)

def atan2d(x,y)
	return atan2(x*pi/180.0, y*pi/180.0)



#Main function execution starts here
setup()
initialize()



#Variables Initialization
diff_sum = 0;

for i in range(1,num_iteration):
	print i
	x_hat_k = x_hat[:,i-1]
	angle_bias[i] = angle_bias[i-1] + phi
	bias = angle_bias[i]
	previous_alpha_bias = scan_radius*sind(bias-phi)
	previous_beta_bias = scan_radius*cosd(bias-phi)
	alpha_bias = scan_radius*sind(bias)
	beta_bias = scan_radius*cosd(bias)
    
	alpha_diff = alpha_bias - previous_alpha_bias
	beta_diff = beta_bias - previous_beta_bias
    
	previous_u = [u2,u3]
	scan_parameters = [scan_radius, bias, phi]
    
	C = get_C_matrix(x_hat_k,previous_u,scan_parameters)
	P_current = A*P_current*A + Q

    #Output Calculation
	measurement = getIntensity()
	y = [[measurement],[previous_measurement],[previous_previous_measurement]]
	y_hat = get_output_array(x_hat_k, previous_u,scan_parameters)
	previous_measurement = measurement
	previous_previous_measurement = previous_measurement

	#Filtering    
    K = P_current*np.transpose(C)*linalg.inverse(C*P_current*np.transpose(C) + R)
    x_hat[:,i] = x_hat_k+K*(y-y_hat)
    P_current = (np.identity(3) - K*C)*P_current

    difference = abs(y[0]-y_hat[0])
    diff_sum = diff_sum + difference
    
    if x_hat[1,i] < 0:
        x_hat_k[1,i] = 0
   
    
    if(difference + previous_difference < 2):
        G = 0.2
        G2 = 0.2
    else:
        G = 0.1
        G2 = 0
    
    #G = 0.0
    previous_difference = difference;
    
    normal_u2 = -G*x_hat[1,i]
    normal_u3 = -G*x_hat[2,i]
    
    u2 = [[normal_u2], [u2_previous]]
    u3 = [[normal_u2], [u3_previous]]

    u2_previous = normal_u2
    u3_previous = normal_u3
    
    psi(i) = psi(i-1) + normal_u2
    theta(i) = theta(i-1) + normal_u3
    
    x_hat_k[:,i] = x_hat_k[:,i] + [[0],[normal_u2],[normal_u3]]

    alpha_u = alpha_bias*pi/180.0
    beta_u = beta_bias*pi/180.0
 
    psi_offset = atan2d(cos(alpha_u)*sin(beta_u), cosd(alpha_u)*cosd(beta_u)*cosd(theta[i]) - \
    	sind(alpha_u)*cosd(theta[i]))
    theta_offset = asind(cosd(alpha_u)*cosd(beta_u)*sind(theta[i]) + sind(alpha_u)*cosd(theta[i]))-theta[i]
    
    scan_psi[i] = psi[i] + psi_offset
    scan_theta[i] = theta[i] + theta_offset
    
    Motor_command_stepper = scan_psi[i] - scan_psi[i-1]
    Motor_command_servo = scan_theta[i]

    myServo.write(Motor_command_servo)
    myStepper.rotateMotor(Motor_command_stepper)