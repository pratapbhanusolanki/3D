import mraa

sensorPin = mraa.Aio(3)

def getIntensity():
	total_intensity = 0
	for inum in range(1,100):
		v = (sensorPin.readFloat()/1024.0)*7.6
		total_intensity = total_intensity + v

	avg_intensity = total_intensity/inum
	return avg_intensity

while(1):
	print 'Press enter for new measurement'
	input = raw_input(">> ")
	measurement = getIntensity()
	print 'Measurement = %f V' %(measurement)

