#   PROGRAM-SKRYPT PYTHON DLA RPI 3B+
#   MINI-SYSTEM WIZYJNY
#   ROBOTA TYPU DELTA "ASTRA*"
#   WIKTOR NOWACKI wik2001(at)windowslive.com
#
#   Wykorzystano fragmenty kodow z kursow Forbot.pl
#   UART w Python'ie: https://www.elinux.org/Serial_port_programming
#   Kamera + OpenCV + RPi: https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/
#
#   KOD UDOSTEPNIONO "AS IS" WYLACZNIE W CELACH EDUKACYJNYCH!
#   
#   Czesc komentarzy pochodza z w/w stron, pozostaje moje - czesc po polsku, czesc anglojezyczna

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy
import serial

k1=''
k2=''
k3=''

port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.5)

#while True:
#    port.write(('a').encode())
#    time.sleep(1)

#adding text
font = cv2.FONT_HERSHEY_SIMPLEX

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

#while True:
#port.write(("abcd").encode())
#debugowanie komunikacji UART Nucleo<->RPi

while True:
    rcv = port.read(1)
    if rcv==('q').encode():
        #port.write(("OK :)").encode())
        # grab an image from the camera
        time.sleep(0.1)
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array

        # display the image on screen and wait for a keypress
        #cv2.imshow("Image", image)
        #time.sleep(1)
        #cv2.destroyAllWindows()
        #height, width = image.shape[:2]
        #print("widht=",width," height=",height)

        crop_img1 = image[20:20+60, 530-60:530]
        avg_color_per_row1 = numpy.average(crop_img1, axis=0)
        b1,g1,r1 = numpy.average(avg_color_per_row1, axis=0)
        print(b1, g1, r1)
        #cv2.putText(crop_img1,'1',(5,50), font, 2,(255,255,255),4,cv2.LINE_AA)
        #cv2.imshow("cropped1", crop_img1)

        crop_img2 = image[160:160+60, 500-60:500]
        avg_color_per_row2 = numpy.average(crop_img2, axis=0)
        b2,g2,r2 = numpy.average(avg_color_per_row2, axis=0)
        print(b2,g2,r2)
        #cv2.putText(crop_img2,'2',(5,50), font, 2,(255,255,255),4,cv2.LINE_AA)
        #cv2.imshow("cropped2", crop_img2)

        crop_img3 = image[250:250+60, 560-60:560]
        avg_color_per_row3 = numpy.average(crop_img3, axis=0)
        b3,g3,r3 = numpy.average(avg_color_per_row3, axis=0)
        print(b3,g3,r3)
        #cv2.putText(crop_img3,'3',(5,50), font, 2,(255,255,255),4,cv2.LINE_AA)
        #cv2.imshow("cropped3", crop_img3)
        
        granica = 205

        if b1>=granica:
            print('1 - bialy')
            k1 = 'b'
        elif g1>=granica:
            print('1 - zolty')
            k1 = 'z'
        else:
            print('1 - czerwony')
            k1 = 'c'

        if b2>=granica:
            print('2 - bialy')
            k2 = 'b'
        elif g2>=granica:
            print('2 - zolty')
            k2 = 'z'
        else:
            print('2 - czerwony')
            k2 = 'c'

        if b3>=granica:
            print('3 - bialy')
            k3 = 'b'
        elif g3>=granica:
            print('3 - zolty')
            k3 = 'z'
        else:
            print('3 - czerwony')
            k3 = 'c'
        #time.sleep(0.1)
        
        rawCapture.truncate(0)
        cmd=k1+k2+k3
        port.write(cmd.encode())
