#This file creates a 5V 50% dutycycle pulse on pin PIN_NUM
import mraa
import time

def usleep(x): 
	time.sleep(x/1000000.0)
PIN_NUM = 8
SLEEP_TIME = 0
LED_IN = mraa.Gpio(PIN_NUM)  
LED_IN.dir(mraa.DIR_OUT) 

while(1):
	LED_IN.write(0)
	usleep(SLEEP_TIME)
	LED_IN.write(1)
	usleep(SLEEP_TIME)
