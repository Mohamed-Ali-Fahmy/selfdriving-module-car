from picamera.array import PiRGBArray
import numpy as np

from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40, GPIO.OUT) #this pins is used to turn light on  for the camera
GPIO.output(40, GPIO.HIGH)

camera = PiCamera()
camera.resolution = (200, 125) #lower resolution for the computer vision in order to make the process faster
#camera.rotation = 180 # turning the image upside down 
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(200, 120))

time.sleep(0.1) # a short time to make sure that the camera is on

kernel = np.ones((3,3), np.uint8) #matix  is created
x_last = 100
y_last = 80


GPIO.setup(10, GPIO.OUT)  #  set GPIO pin 10  to output mode
p1 = GPIO.PWM(10, 100)   # Initialize PWM on pwmPin 100Hz frequency

GPIO.setup(12, GPIO.OUT)  # Set GPIO pin 12 to output mode.
p2 = GPIO.PWM(12, 100)

GPIO.setup(24, GPIO.OUT)  # Set GPIO pin 24 to output mode.
p3 = GPIO.PWM(24, 100)   # Initialize PWM on pwmPin 100Hz frequency

GPIO.setup(26, GPIO.OUT)  # Set GPIO pin 26 to output mode.
p4 = GPIO.PWM(26, 100)

# main loop of program
print("\nPress Ctl C to quit \n")  # Print blank line before and after message.
dc=0                               # set dc variable to 0 for 0%
p1.start(dc)
p2.start(dc) # Start PWM with 0% duty cycle
p3.start(dc)
p4.start(dc)


def Motor_speed(speed, steering):
    if steering == 0:
     p3.ChangeDutyCycle(speed)
     p1.ChangeDutyCycle(speed)
     return
    elif steering > 0:
     steering = 100 - steering
     p3.ChangeDutyCycle(speed)
     p1.ChangeDutyCycle(speed*steering/100)
     return
    elif steering < 0:
     steering = steering * -1
     steering = 100 - steering
     p3.ChangeDutyCycle(speed*steering/100)
     p1.ChangeDutyCycle(speed)
     return


kp = 0.75
ap = 1

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array  # save the array, that holds the image, in a variable called image
    image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    _,image_Binary = cv2.threshold(image_gray, 150 ,255,  cv2.THRESH_BINARY_INV) # any color between 4 to 255 is set to be white other wise is black

   #Blackline = cv2.inRange(image_Binary, 0)
    image_Binary = cv2.erode(image_Binary, kernel, iterations=5) #this function helps to get ride of oise for example a very small white parts.
    image_Binary = cv2.dilate(image_Binary, kernel, iterations=4) #this also used to remove noise
   # rio = image_Binary[200:250,0:639]
    blk_line, contours_blk, hierarchy_blk = cv2.findContours( image_Binary.copy() , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #contour mode

    cv2.drawContours(image, contours_blk, -1, (0, 20,0), 3) #########
    contours_blk_len = len(contours_blk)
    if contours_blk_len > 0 : # if the program found some contours. if the length of the contour in bigger that zero
     if contours_blk_len == 1 : # if it found one contour, so we are luck and the program will jump to many calculation
      blackbox = cv2.minAreaRect(contours_blk[0]) # find the rectangle with the mini area around the first detected contour.
     else: ##In case of many detected contours we have to identify which one should we choose.
       canditates=[]
       off_bottom = 0
       for con_num in range(contours_blk_len): # looping over the first detected contours
        blackbox = cv2.minAreaRect(contours_blk[con_num])
        (x_min, y_min), (w_min, h_min), ang = blackbox ##saving value from the box around the contour
        box = cv2.boxPoints(blackbox)
        (x_box,y_box) = box[0]
        if y_box > 118 :
         off_bottom += 1
        canditates.append((y_box,con_num,x_min,y_min))
       canditates = sorted(canditates)
       if off_bottom > 1:
        canditates_off_bottom=[]
        for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
           (y_highest,con_highest,x_min, y_min) = canditates[con_num]
           total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
           canditates_off_bottom.append((total_distance,con_highest))
        canditates_off_bottom = sorted(canditates_off_bottom)
        (total_distance,con_highest) = canditates_off_bottom[0]
        blackbox = cv2.minAreaRect(contours_blk[con_highest])
       else:
        (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]
        blackbox = cv2.minAreaRect(contours_blk[con_highest])


     # continue my Program
     (x_min, y_min), (w_min, h_min), ang = blackbox # x_mi, y_min is tthe point on the center of the drawm rectangle
     x_last = x_min
     y_last = y_min
     if ang < -45 :
      ang = 90 + ang
     if w_min < h_min and ang > 0:
      ang = (90-ang)*-1
     if w_min > h_min and ang < 0:
      ang = 90 + ang
     setpoint = 100 ####### setpoint is used to make origin point out of x_min
     error = int(x_min - setpoint)
     ang = int(ang)
     Motor_speed(45,(error*kp)+(ang*ap))
     box = cv2.boxPoints(blackbox) #obtaining points that serounds the minAreaRect to be able to draw it as a contour in the image
     box = np.int0(box)
     cv2.drawContours(image,[box],0,(0,0,255),3) #draw the contour to the image so that we can see it
     cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
     cv2.putText(image,str(error),(10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
     cv2.line(image, (int(x_min),90 ), (int(x_min),110 ), (255,0,0),3)


    cv2.imshow("orginal with line", image_Binary) #shows the variale image that holds the array of the image.
    rawCapture.truncate(0)  # the function truncate is used here to empty the buffer sothat the next fram can be written to it.
    key = cv2.waitKey(1) & 0xFF  # read a key
    if key == ord("q"): # if the pressed key is the charachter q then the loop breaks and finishes
        break

GPIO.output(40, GPIO.LOW) # turn the light of the camera off
