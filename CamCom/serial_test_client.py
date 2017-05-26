import time
import serial
import mraa
import cv2.cv as cv
import cv2
import numpy
import struct
from xmodem import XMODEM


scale = 0.2
flag = 0

while flag==0:
    try:
        cap = cv.CaptureFromCAM(0)
        img = cv.QueryFrame(cap)
        #cap.release()
        img_array = cv2.resize(numpy.asarray(img[:,:]), (0,0), fx=scale, fy=scale)
        flag = 1
        print "Image capturing success"
        cv.SaveImage("cam.jpg", img)
    except:
        print "Image capturing failed, trying again"
        flag = 0

shape_img = img_array.shape
print shape_img

# configure the serial connections (the parameters differs on the device you are connecting to)

uart = mraa.Uart(0)

ser = serial.Serial(
    port=uart.getDevicePath(),
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    #timeout=0
)


val = 233




print 'Serial started \n'

while 1 :
    # send the character to the device
    # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
    out = ''
    # let's wait one second before reading output (let's give device time to answer)
    time.sleep(1)
    while ser.inWaiting() > 0:
        out += ser.read(1)

    if out != '':
        start = time.time()
        print "Capturing Image ...."
        #cap = cv.CaptureFromCAM(0)
        flag = 0
        while flag==0:
            try:
                cap = cv.CaptureFromCAM(0)
                img = cv.QueryFrame(cap)
                img_array = cv2.resize(numpy.asarray(img[:,:]), (0,0), fx=scale, fy=scale)
                #cap.release()

                flag = 1
                print "Image capturing success"
                cv.SaveImage("cam_transmitted.jpg", img)
            except:
                print "Image capturing failed, trying again"
                flag = 0
        print "Transmitting Image ...."
        # for i in range(0,shape_img[0]):
        #     for j in range(0,shape_img[1]):
        #         for k in range(0,shape_img[2]):
        #             val = img_array[i,j,k]
        #             ser.write(str(val)+'\n')
        stream = open('cam_transmitted.jpg', 'rb')
        transmitted_data = stream.read()
        #print modem.send('Bhanu')
        ser.write(transmitted_data)
        print len(transmitted_data)
        #ser.write("\n<<EOF>>\n")
        toc = time.time()
        interval = toc - start
        print "Took " + str(interval) + " seconds to transmit the image" 
        break


