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
 
#//////////////////////////////////////////////////////////////////////////////
 
def setup():
  AIN1 = 3
  AIN2 = 5
  BIN2 = 6

  global GimbalRed
  GimbalRed = mraa.Pwm(AIN1)
  GimbalRed.period_us(32)
  GimbalRed.enable(True)

  global GimbalOrange
  GimbalOrange = mraa.Pwm(AIN2)
  GimbalOrange.period_us(32)
  GimbalOrange.enable(True)
  
  global GimbalBrown
  GimbalBrown = mraa.Pwm(BIN2)
  GimbalBrown.period_us(32)
  GimbalBrown.enable(True)
  
  global PWMA
  PWMA = mraa.Gpio(13)  
  PWMA.dir(mraa.DIR_OUT)
  PWMA.write(1)

  global PWMB
  PWMB = mraa.Gpio(12)  
  PWMB.dir(mraa.DIR_OUT)
  PWMB.write(1)

  global STBY
  STBY = mraa.Gpio(8)  
  STBY.dir(mraa.DIR_OUT)
  STBY.write(1)

  global pwmSin
  pwmSin = [139,151,163,174,185,195,205,214,223,230,237,242,247,250,253,254,254,253,251,248,244,239,232,225,217,208,199,188,178,166,155,143,131,119,107,95,84,73,62,52,43,34,26,19,13,8,5,2,0,0,0,2,5,8,13,19,26,34,43,52,62,73,84,95,107,119]
  #pwmSin = [128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109]
  #pwmSin = [128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124]
  
  global sineArraySize
  sineArraySize = len(pwmSin)                 #Find lookup table size
  phaseShift = sineArraySize/3          #Find phase shift and initial A, B C phase values

  global currentStepRed
  currentStepRed = 0

  global currentStepOrange
  currentStepOrange = currentStepRed + phaseShift

  global currentStepBrown
  currentStepBrown = currentStepOrange + phaseShift
 
  sineArraySize = sineArraySize-1 # Convert from array Size to last PWM array number
 
#//////////////////////////////////////////////////////////////////////////////
num_iteration = 200
setup()
direct = 1
for i in range(1,num_iteration):
  GimbalRed.write(pwmSin[currentStepRed]/255.0)
  GimbalOrange.write(pwmSin[currentStepOrange]/255.0)
  GimbalBrown.write(pwmSin[currentStepBrown]/255.0)  
  
  if (direct==1): 
    increment = 1
  else:
    increment = -1    
 
  currentStepRed = currentStepRed + increment;
  currentStepOrange = currentStepOrange + increment;
  currentStepBrown = currentStepBrown + increment;
 
  #Check for lookup table overflow and return to opposite end if necessary
  if(currentStepRed > sineArraySize):
    currentStepRed = 0
  if(currentStepRed < 0):
    currentStepRed = sineArraySize;
 
  if(currentStepOrange > sineArraySize):
    currentStepOrange = 0
  if(currentStepOrange < 0):
    currentStepOrange = sineArraySize
 
  if(currentStepBrown > sineArraySize):
    currentStepBrown = 0
  if(currentStepBrown < 0):
    currentStepBrown = sineArraySize
  
  #Control speed by this delay
  time.sleep(0.5)
  print i