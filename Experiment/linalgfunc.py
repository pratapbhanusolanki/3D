#Contains linear Algebric Functions to be used in the project
import math
import numpy as np
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

def get_C_matrix(x,previous_u,scan_parameters):
    C = np.zeros((3,3))
    u2 = previous_u[0,:]
    u3 = previous_u[1,:]
    scan_radius = scan_parameters[0]
    bias = scan_parameters[1]
    phi = scan_parameters[2]
    scaling_coefficient = 1
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi])
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) 
    x1 = x[0]
    x2 = x[1] + beta_biases[0]
    x3 = x[2] + alpha_biases[0]
    C[0,:] = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]

    #previous values
    x1 = x[0]
    x2 = x[1]-u2[0] + beta_biases[1]
    x3 = x[2]-u3[0] + alpha_biases[1]
    C[1,:] = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]

    #Previous to preious values
    x1 = x[0]
    x2 = x[1]-u2[0]-u2[1] + beta_biases[2]
    x3 = x[2]-u3[0]-u3[1] + alpha_biases[2]
    C[2,:] = [g(x1)*g(x2), x1*g_d(x2)*g(x3), x1*g(x2)*g_d(x3)]
    #pdb.set_trace()
    Adum = np.matrix([[1,0,0],[0,scaling_coefficient,0],[0,0,scaling_coefficient]])
    y = C*Adum
    return y

def get_output_array(x,previous_u,scan_parameters):
    u2 = previous_u[0,:]
    u3 = previous_u[1,:]
    scan_radius = scan_parameters[0]
    bias = scan_parameters[1]
    phi = scan_parameters[2]
    scaling_coefficient = 1
    alpha_biases = scan_radius*sind([bias,bias-phi, bias-2*phi])
    beta_biases = scan_radius*cosd([bias,bias-phi, bias-2*phi]) 
    x1 = x[0]
    x2 = x[1] + beta_biases[0]
    x3 = x[2] + alpha_biases[0]
    y1 = x1*g(x2)*g(x3)

    #Previous values
    x1 = x[0]
    x2 = x[1]-u2[0] + beta_biases[1]
    x3 = x[2]-u3[0] + alpha_biases[1]
    y2 = x1*g(x2)*g(x3)

    #Previous to previous values
    x1 = x[0]
    x2 = x[1]-u2[0]-u2[1] + beta_biases[2]
    x3 = x[2]-u3[0]-u3[1] + alpha_biases[2]
    y3 = x1*g(x2)*g(x3)
    y = np.array([[y1],[y2],[y3]])
    return y

#Gives Fitting Gaussian data from the module 
def g(x):
    a1 = 0.7979
    b1 =-0.7089
    c1 = 8.257
    a2 = 0.2016
    b2 =-1.076
    c2 = 29.78
    arg1 = np.power((x-b1)/c1,2)
    arg2 = np.power((x-b2)/c2,2)
    y=a1*np.exp(-arg1) + a2*np.exp(-arg2)
    return y

#computes the derivative of the gaussian function y = g_d(x)
def g_d(x):
    a1 = 0.7979
    b1 =-0.7089
    c1 = 8.257
    a2 = 0.2016
    b2 =-1.076
    c2 = 29.78
    arg1 = np.power((x-b1)/c1,2)
    arg2 = np.power((x-b2)/c2,2)
    y = -2*a1*((x-b1)/(c1*c1))*np.exp(-arg1) -2*a2*((x-b2)/(c2*c2))*np.exp(-arg2)
    return y

def gaussianValue(x):
    a1 = 0.7979
    b1 =-0.7089
    c1 = 8.257
    a2 = 0.2016
    b2 =-1.076
    c2 = 29.78
    y = a1*math.exp(-((x-b1)/c1)**2) + a2*math.exp(-((x-b2)/c2)**2)

def gaussianDerivative(x):
    a1 = 0.7979
    b1 =-0.7089
    c1 = 8.257
    a2 = 0.2016
    b2 =-1.076
    c2 = 29.78
    y = -2*a1*((x-b1)/c1**2)*math.exp(-((x-b1)/c1)**2) -2*a2*((x-b2)/c2**2)*math.exp(-((x-b2)/c2)**2)