import time
import serial
import mraa
import cv2.cv as cv
import cv2
import numpy
import struct

def get_image_bytes(num=1,size = 20):
    image_name = "cam_"+ str(num) + ".jpg"
    stream = open(image_name, 'rb')
    data = stream.read()
    #print data[0:size]
    return data[0:size]

scale = 0.1
flag = 0

for num in range(1,7):
    print num
    raw_input("Press Enter to continue...")
    print "marker"
    flag = 0
    while flag==0:
        try:
            cap = cv.CaptureFromCAM(0)
            img = cv.QueryFrame(cap)
            #cap.release()
            img_array = cv2.resize(numpy.asarray(img[:,:]), (0,0), fx=scale, fy=scale)
            flag = 1
            print "Image capturing success"
            image_name = "cam_"+ str(num) + ".jpg"
            cv.SaveImage(image_name, img)
        except:
            print "Image capturing failed, trying again"
            flag = 0

shape_img = img_array.shape
print shape_img

