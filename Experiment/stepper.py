import mraa  
import time  

def usleep(x): 
	time.sleep(x/1000000.0)

#########################
##### Steptype Class ########
#########################

class StepType:
    def __init__(self,name, MS1, MS2, MS3, factor):
        self.name = name
        self.MS1 = MS1
        self.MS2 = MS2
        self.MS3 = MS3
        self.factor = factor

global full_step
full_step = StepType('full_step',0,0,0,1)
global half_step
half_step = StepType('half_step',1,0,0,2)
global quarter_step
quarter_step = StepType('quarter_step',0,1,0,4)
global eighth_step
eighth_step = StepType('eighth_step',1,1,0,8)
global Sixteenth_step
sixteenth_step = StepType('sixteenth_step',1,1,1,16) 


class Stepper:
	def __init__(self,pindir,pinstep,pinenable,type_name):    #10,11, 9 
		self.step = mraa.Gpio(pinstep)
		self.step.dir(mraa.DIR_OUT)
		self.direction = mraa.Gpio(pindir)
		self.direction.dir(mraa.DIR_OUT)
		self.type_name = type_name
		self.MS1 = mraa.Gpio(3)
		self.MS1.dir(mraa.DIR_OUT)
		self.MS2 = mraa.Gpio(4)
		self.MS2.dir(mraa.DIR_OUT)
		self.MS3 = mraa.Gpio(5)
		self.MS3.dir(mraa.DIR_OUT)
		self.enable = mraa.Gpio(pinenable)
		self.enable.dir(mraa.DIR_OUT)

		if type_name == 'full_step':
			current_step_type = full_step
		elif type_name == 'half_step':
			current_step_type = half_step
		elif type_name == 'quarter_step':
			current_step_type = quarter_step
		elif type_name == 'eighth_step':
			current_step_type = eighth_step
		elif type_name == 'sixteenth_step':
			current_step_type = sixteenth_step
		self.step_type = current_step_type
		self.resetPins()

	def resetPins(self):
		self.step.write(0)
		self.direction.write(0)
		current_step_type = self.step_type
		self.MS1.write(current_step_type.MS1)
		self.MS2.write(current_step_type.MS2)
		self.MS3.write(current_step_type.MS2)
		self.enable.write(1)

	def rotateMotor(self, angle):
		#print 'check_1'
		factor = self.step_type.factor
		steps = int((angle*403*factor)/(360))
		#print steps
		self.rotateStep(steps)

   	def rotateStep(self,steps):
   		#print 'check_2'
   		if steps > 0:
			self.direction.write(0)  #Pull direction pin low to move "forward"
		else:
			self.direction.write(1)	#Pull direction pin high to move "backwards"

		step_count = abs(steps)
		self.enable.write(0) ##Pull enable pin low to set FETs active and allow motor control
		#print 'check_3, enabling to 0'
		for x in range(1,step_count):
			self.step.write(1) #Trigger one step forward
			usleep(1000)
			self.step.write(0) #Pull step pin low so it can be triggered again
			usleep(1000)
		#self.resetPins()
		#print 'check_4, resetting pins to 0'
