from stepper import Stepper
import stepper
global ReceiverStepper
import mraa

global sensorPin
sensorPin = mraa.Aio(0)

def getIntensity():
	total_intensity = 0
	for inum in range(1,100):
		v = (sensorPin.read()/1024.0)*7.6
		total_intensity = total_intensity + v

	avg_intensity = total_intensity/inum
	return avg_intensity


print 'Choose one of the following steppers: \n1.Base \n2.Receiver'
input = raw_input(">> ")
if int(input) == 1:
	Stepper = Stepper(10,11,9,'full_step')
else:
	Stepper = Stepper(7,6,2,'full_step')

while(1):
	print 'Enter the angle in degree to rotate the stepper'
	input = raw_input(">> ")
	Stepper.rotateMotor(int(input))
	s = getIntensity()
	print 'Measuremnt: %f' %(s)
	#Stepper.rotateMotor(-int(input))
