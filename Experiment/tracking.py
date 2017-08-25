import mraa  
import time
import datetime  
import subprocess
import numpy as np
from numpy import linalg
import math
import scipy as sio
from stepper import Stepper
import stepper
import linalgfunc
import pdb
import os
import paramiko 
paramiko.util.log_to_file('/tmp/paramiko.log')

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
	global Light
	Light = mraa.Gpio(8)  
	Light.dir(mraa.DIR_OUT) 

	global sensorPin
	sensorPin = mraa.Aio(5)

	global BaseStepper
	BaseStepper = Stepper(10,11,9,'eighth_step')

	global ReceiverStepper
	ReceiverStepper = Stepper(7,6,2,'eighth_step')

	global initial_pitch 
	initial_pitch = 45
	ReceiverStepper.rotateMotor(initial_pitch)

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
	Q = 0.1*Q

	global R
	R = 0.11

	global P_current
	P_current = np.identity(3)

	global scan_parameters_all
	scan_parameters_all = np.zeros((num_iteration,3))

	global previous_u_all
	previous_u_all = np.zeros((num_iteration,2,2))

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

	global motor_commands_all
	motor_commands_all = np.zeros((num_iteration,2))


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
diff_sum = 0
base_sum_angle = 0
receiver_sum_angle = initial_pitch
x_hat = np.zeros((3,num_iteration))
x_hat[:,0] = [3,0,0]
x_hatf_all[0,:] = x_hat[:,0]
angle_bias = np.zeros(num_iteration) 
phi = 30
scan_radius = 10
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
theta[0] = 10
scan_theta[0] = theta[0]
ReceiverStepper.rotateMotor(-theta[0])
receiver_sum_angle = receiver_sum_angle -theta[0]
onLights()
time.sleep(1)
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
	previous_u_all[i,:,:] = previous_u.T
	#print previous_u
	scan_parameters = [scan_radius, bias, phi]
	scan_parameters_all[i,:] = scan_parameters
	#print scan_parameters
    
	
	C = linalgfunc.get_C_matrix(x_hat_k,previous_u,scan_parameters)
	C_all[i,:,:] = C
	#print C

	P_current = A*P_current*A + Q
	Pf_all[i,:,:] = P_current
	#print P_current
    #Output Calculation
	measurement = getIntensity()
	y = measurement
	y_all[i] = y
	#print y

	y_hat = linalgfunc.get_output_array(x_hat_k, previous_u,scan_parameters)
	y_hat_all[i] = np.transpose(y_hat)
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

	difference = abs(y-y_hat)
	diff_sum = diff_sum + difference
	
	if x_hat[0,i] < 0:
		x_hat[0,i] = 0
	x_hat_all[i,:] = x_hat[:,i]
    
	if(difference + previous_difference < 1):
		G = 0.2
		G2 = 0.2
	else:
		G = 0.0
		G2 = 0

	G = 0.0
	# G2 = 0
	# G=0.2

	previous_difference = difference
	normal_u2 = -G*x_hat[1,i]
	normal_u3 = -G*x_hat[2,i]
	u_all[i,:] = [0,normal_u2,normal_u3]
	
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
	theta_offset = theta_offset_temp - theta[i]
   
	scan_psi[i] = psi[i] + psi_offset
	scan_theta[i] = theta[i] + theta_offset

    # u2 = np.array([[normal_u2], [u2_previous]])
	# u3 = np.array([[normal_u3], [u3_previous]])

	# u2_previous = normal_u2
	# u3_previous = normal_u3

	Motor_command_base = scan_psi[i] - scan_psi[i-1]
	Motor_command_receiver = scan_theta[i] - scan_theta[i-1]
	#Motor_command_receiver = 0
	print Motor_command_base 
	print -Motor_command_receiver
	motor_commands_all[i,0] = Motor_command_base
	motor_commands_all[i,1] = Motor_command_receiver
	BaseStepper.rotateMotor(Motor_command_base)
	ReceiverStepper.rotateMotor(-Motor_command_receiver)
	base_sum_angle = base_sum_angle + Motor_command_base
	receiver_sum_angle = receiver_sum_angle + Motor_command_receiver
	toc = time.time()
	timer[i] = toc-start
	#raw_input("Press Enter to continue...")
	#time.sleep(0.5)
BaseStepper.rotateMotor(-base_sum_angle)
ReceiverStepper.rotateMotor(-receiver_sum_angle)
st = datetime.datetime.fromtimestamp(toc).strftime('%Y-%m-%d_%H:%M:%S')
zip_name = datetime.datetime.fromtimestamp(toc).strftime('data_%Y-%m-%d_%H:%M:%S.npz') 

np.savez(zip_name, scan_parameters_all=scan_parameters_all, \
	x_hatf_all=x_hatf_all, x_hat=x_hat, Pf_all=Pf_all,\
	C_all=C_all, x_hat_all=x_hat_all, y_hat_all=y_hat_all,\
	y_all=y_all, P_all=P_all, K_all=K_all, timer=timer, u_all=u_all,\
	scan_psi_all=scan_psi,scan_theta_all=scan_theta, previous_u_all=previous_u_all,\
	motor_commands_all=motor_commands_all)

a = subprocess.check_output("who")
user_hostname = a[a.find("(")+1:a.find(")")]

# ssh_zip_name  = datetime.datetime.fromtimestamp(toc).strftime('data_%Y-%m-%d_%H\:%M\:%S.npz')
# scp_command = "scp ./" + ssh_zip_name +" prabhanu@"+ user_hostname + ":Data"
# sent_status = os.system(scp_command)

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(user_hostname, username='prabhanu')
sftp = ssh.open_sftp()

sftp.put(zip_name,'GoogleDriveOld/MSU/Research/AlignmentOpticalCommunication/3D/Experiment/Data/' + zip_name)
