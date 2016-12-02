#Contains linear Algebric Functions to be used in the project
import math

global pi
pi = math.pi

global sin
sin = math.sin

global cos
cos = math.cos

global atan2
atan2 = math.atan2

def tand(x):
    return tan(x*pi/180.0)

def sind(x):
    return sin(x*pi/180.0)

def cosd(x):
    return cos(x*pi/180.0)

def atan2d(x,y):
	return atan2(x*pi/180.0, y*pi/180.0)


def get_C_matrix(x,previous_u,scan_parameters):
    u2 = previous_u[:,1]
    u3 = previous_u[:,2]
    
    scan_radius = scan_parameters[1]
    bias = scan_parameters[2]
    phi = scan_parameters[3]
    
    scaling_coefficient = 1
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi])
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) 
    
	x1 = x[1]
	x2 = x[2] + beta_biases[1]
	x3 = x[3] + alpha_biases[1]
    
    C[1,:] = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]

	%previous values
	x1 = x[1]
	x2 = x[2]-u2[1] + beta_biases[2]
	x3 = x[3]-u3[1] + alpha_biases[2]

	C[2,:] = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]

	%Previous to preious values
	x1 = x(1);
	x2 = x(2)-u2(1)-u2(2) + beta_biases(3)
	x3 = x(3)-u3(1)-u3(2) + alpha_biases(3)

	C(3,:) = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]
    
	y = C*[1,0,0; 0,scaling_coefficient,0 ; 0,0,scaling_coefficient]
	return y

def get_output_array(x,previous_u,scan_parameters)
    u2 = previous_u[:,1]
    u3 = previous_u[:,2]
    
    scan_radius = scan_parameters[1]
    bias = scan_parameters[2]
    phi = scan_parameters[3]
    
    scaling_coefficient = 1
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi])
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) 
	
	#Current values
	x1 = x[1]
	x2 = x[2] + beta_biases[1]
	x3 = x[3] + alpha_biases[1]
	y1 = x1*g(x2)*g(x3)
	
	#Previous values
	x1 = x[1]
	x2 = x[2]-u2[1] + beta_biases[2]
	x3 = x[3]-u3[1] + alpha_biases[2]
	y2 = x1*g(x2)*g(x3)
    
	#Previous to previous values
	x1 = x[1]
	x2 = x[2]-u2[1]-u2[2] + beta_biases[3]
	x3 = x[3]-u3[1]-u3[2] + alpha_biases[3]
	y3 = x1*g(x2)*g(x3)
	y = [[y1],[y2],[y3]]

	return y

#Gives Fitting Gaussian data from the module 
def g(x)
   a1 =      0.6922/1.0359
   b1 =       7.752
   c1 =     148.8
   a2 =       0.346/1.0359
   b2 =      -13.57
   c2 =       325.8
    x = x*18
    y=a1*math.exp(-((x-b1)/c1).^2) + a2*math.exp(-((x-b2)/c2).^2)
    return y

#computes the derivative of the gaussian function y = g_d(x)
def g_d(x)
   a1 =      0.6922/1.0359
   b1 =       7.752
   c1 =     148.8
   a2 =       0.346/1.0359
   b2 =      -13.57
   c2 =       325.8
   x = x*18

   y = -2*a1*((x-b1)/c1^2)*math.exp(-((x-b1)/c1)^2) -2*a2*((x-b2)/c2^2)*math.exp(-((x-b2)/c2)^2)
   y = 18*y
end




