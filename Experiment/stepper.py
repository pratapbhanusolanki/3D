import mraa  
import time  

def usleep(x): 
	time.sleep(x/1000000.0)

class Stepper:
	def __init__(self):
		self.step = mraa.Gpio(6)
		self.step.dir(mraa.DIR_OUT)
		self.direction = mraa.Gpio(7)
		self.direction.dir(mraa.DIR_OUT)
		self.MS1 = mraa.Gpio(3)
		self.MS1.dir(mraa.DIR_OUT)
		self.MS2 = mraa.Gpio(4)
		self.MS2.dir(mraa.DIR_OUT)
		self.MS3 = mraa.Gpio(5)
		self.MS3.dir(mraa.DIR_OUT)
		self.enable = mraa.Gpio(2)
		self.enable.dir(mraa.DIR_OUT)
		self.resetPins()

	def resetPins(self):
		self.step.write(0)
		self.direction.write(0)
		self.MS1.write(1)
		self.MS2.write(1)
		self.MS3.write(1)
		self.enable.write(1)

	def rotateMotor(self, angle):
		steps = int((angle*403*16)/360) 
		self.rotateStep(steps)

   	def rotateStep(self,steps):
   		if steps > 0:
			self.direction.write(1)  #Pull direction pin low to move "forward"
		else:
			self.direction.write(0)	#Pull direction pin high to move "backwards"

		step_count = abs(steps)
		self.enable.write(0) ##Pull enable pin low to set FETs active and allow motor control
		for x in range(1,step_count):
			self.step.write(1) #Trigger one step forward
			usleep(100)
			self.step.write(0) #Pull step pin low so it can be triggered again
			usleep(100)