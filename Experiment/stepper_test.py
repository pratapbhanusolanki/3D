from stepper import Stepper
import stepper
global ReceiverStepper
print 'Choose one of the following steppers: \n1.Base \n2.Receiver'
input = raw_input(">> ")
if int(input) ==1:
	Stepper = Stepper(10,11,9,'full_step')
else:
	Stepper = Stepper(7,6,2,'full_step')

while(1):
	print 'Enter the angle in degree to rotate the stepper'
	input = raw_input(">> ")
	Stepper.rotateMotor(int(input))
	#Stepper.rotateMotor(-int(input))
