import time
import serial
import cv2.cv as cv
import cv2
import numpy
import struct
import matplotlib.pyplot as plt
import matplotlib.image as mpimg



# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=19200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=60.0
)

ser.isOpen()

print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

input=1
#while 1 :
# get keyboard input
input = raw_input(">> ")
    # Python 3 users
    # input = input(">> ")
if input == 'exit':
    ser.close()
    exit()
else:
    # send the character to the device
    # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
    ser.write(input + '\r\n')
    out = ''
    # let's wait one second before reading output (let's give device time to answer)
    #time.sleep(1)
    #out = ser.readline()
    # while ser.inWaiting() > 0:
    #     out += ser.read(1)
    
    # if out != '':
    #     a = out
    #     print ">>" + a
    #     print 0 + int(a)
scale = 0.2
num_cols = 480*scale
num_rows = 640*scale
shape_img = (int(num_cols), int(num_rows), 3)
stream = open('test_received.jpg', 'wb')
header_stream = open('header_jpg.txt','rb')  #Contains first 623 bytes, which are common for all jpegs
header = header_stream.read()
start = time.time()

out = ''
    # let's wait one second before reading output (let's give device time to answer)
time.sleep(1)
while ser.inWaiting() > 0:
    out += ser.read(1)
print out 

# while ser.inWaiting() == 0:
#     num_bytes = 0
# while ser.inWaiting()-num_bytes > 0:
#     num_bytes = ser.inWaiting()
ser.write('send_image')
out = ''
    # let's wait one second before reading output (let's give device time to answer)
time.sleep(2)
while ser.inWaiting() > 0:
    out += ser.read(1)
#print out 
    
#num_bytes = 60000
received_data = out#ser.read(num_bytes)
#print modem.recv(stream)
#received_image = numpy.zeros(shape_img)
# for i in range(0,shape_img[0]):
#     for j in range(0,shape_img[1]):
#         for k in range(0,shape_img[2]):
#             kl = k
#             out = ser.readline()
#             received_image[i,j,kl] = int(out)   ived_data
  #RGBtoBGR issue resolved

toc = time.time()
interval = toc - start
print "Took " + str(interval) + " seconds to receive the image data" 

stream.write(header[:623] + received_data + header[-2:] )
print len(received_data)
ser.close()
stream.close()
header_stream.close()
# plt.imshow(received_image)
# plt.show()

