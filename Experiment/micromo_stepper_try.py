import mraa  
import time  
import pdb

def usleep(x): 
	time.sleep(x/1000000.0)


def setup():
	global Input01
	Input01 = mraa.Gpio(2)  
	Input01.dir(mraa.DIR_OUT) 

	global Input02
	Input02 = mraa.Gpio(3)  
	Input02.dir(mraa.DIR_OUT) 

	global Input11
	Input11 = mraa.Gpio(4)  
	Input11.dir(mraa.DIR_OUT) 

	global Input12
	Input12 = mraa.Gpio(5)  
	Input12.dir(mraa.DIR_OUT) 

	global Phase1
	Phase1 = mraa.Gpio(6)  
	Phase1.dir(mraa.DIR_OUT) 

	global Phase2
	Phase2 = mraa.Gpio(7)  
	Phase2.dir(mraa.DIR_OUT) 

setup()
Input01.write(1)
Input02.write(1)
Input11.write(0)
Input12.write(0)
Phase1.write(0)
Phase2.write(0)
print 'Please enter pulse time in microseconds'
input = raw_input(">> ")
time_interval = int(input)
while(1):
	usleep(time_interval)
	Phase1.write(1)
	usleep(time_interval)
	Phase2.write(1)
	usleep(time_interval)
	Phase1.write(0)
	usleep(time_interval)
	Phase2.write(0)





