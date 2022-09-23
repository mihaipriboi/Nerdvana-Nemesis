import math
import serial
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Button
import RPi.GPIO as GPIO
from threading import Thread
import sys
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

GPIO.setmode(GPIO.BOARD)      # Set GPIO mode to BCM
GPIO.setwarnings(False);

factory = PiGPIOFactory()

# Pin numbers
BUTTON = 26  # GPIO pin
# Motor driver
PWMA = 33
AIN1 = 29
AIN2 = 31
STBY = 36
# Servo motor
SERV = 12  # GPIO pin!!!

# Setup Pins
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# Motor driver
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)


button = Button(BUTTON)
# Initialization
servo = Servo(SERV, min_pulse_width=1/1000, max_pulse_width=2/1000, pin_factory=factory)
# Motor driver
pwmFreq = 100
pwma = GPIO.PWM(PWMA, pwmFreq)
pwma.start(100)
    
#        ###############
#        # Functionuri #
#        ###############
 

colorCube = 0
xCube = 0
def imgProcc(frame):
    global colorCube, xCube, yCube, IMAGE_WIDTH,IMAGE_HEIGHT 
    img = cv2.blur(frame,(3,3))
    # show the frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower bound and upper bound for Green color
    lower_boundGreen = np.array([50, 85, 12])   
    upper_boundGreen = np.array([100, 255, 255])
    # find the colors within the boundaries
    maskGreen = cv2.inRange(hsv, lower_boundGreen, upper_boundGreen)
    
    #red
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([5, 255, 255])
     
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])
     
    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)
     
    maskRed = lower_mask + upper_mask;
    
    
    kernel = np.ones((7,7),np.uint8)
    # Remove unnecessary noise from mask
    maskGreen = cv2.morphologyEx(maskGreen, cv2.MORPH_CLOSE, kernel)
    maskGreen = cv2.morphologyEx(maskGreen, cv2.MORPH_OPEN, kernel)
    maskRed = cv2.morphologyEx(maskRed, cv2.MORPH_CLOSE, kernel)
    maskRed = cv2.morphologyEx(maskRed, cv2.MORPH_OPEN, kernel)
    
    contoursGreen, hierarchy = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoursRed, hierarchy = cv2.findContours(maskRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    img = cv2.drawContours(img, contoursRed, -1, (0, 0, 255), 3)
    img = cv2.drawContours(img, contoursGreen, -1, (0, 255, 0), 3)
    minArea = 400
    
    colorCube = ''
    xCube = IMAGE_WIDTH / 2
    yCube = 0
    for green in contoursGreen:
        areaGreen = cv2.contourArea(green)
        if areaGreen > minArea:
            colorCube = 'G'
            M = cv2.moments(green)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            xCube = cx
            yCube = cy
            #print("Coords x and y for green: ", cx, cy)
            cv2.circle(img,(cx,cy),5,(0,255,0),-1)
    for red in contoursRed:
        areaRed = cv2.contourArea(red)
        if areaRed > minArea:
            colorCube = 'R'
            #print("red")
            M = cv2.moments(red)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            xCube = cx
            yCube = cy
            #print("Coords x and y for red: ", cx, cy)
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
    
    return img

def motorStart(speed):
    GPIO.output(STBY, GPIO.HIGH);
    in1 = GPIO.HIGH
    in2 = GPIO.LOW

    if(speed < 0):
        speed = speed * -1
        in1 = GPIO.LOW
        in2 = GPIO.HIGH

    GPIO.output(AIN1, in1)
    GPIO.output(AIN2, in2)
    pwma.ChangeDutyCycle(speed)

def motorStop():
    GPIO.output(STBY, GPIO.LOW)
    
def readRed():
    start = time.time()
    for impulse_count in range(NUM_CYCLES):
        GPIO.wait_for_edge(COLOR_OUT, GPIO.FALLING)
    duration = time.time() - start      #seconds to run for loop
    red = NUM_CYCLES / duration   #in Hz
    #print("red value - ",red)

ultrasonic = "0 0"
def arduinoReceving(ser):
    global ultrasonic, endProgram
    while True:
        if endProgram == True: return 0
        if ser.in_waiting > 0:
            ultrasonic = ser.readline().decode('utf-8', 'ignore').rstrip()
            

def readSensors():
    global ultrasonic
    
    leftSensor = 0
    rightSensor = 0
    while leftSensor * rightSensor == 0:
        sensor = ultrasonic.split()
        leftSensor = int(sensor[0])
        if len(sensor) > 1:  # ca sa nu avem erori daca citeste doar un nr
            rightSensor = int(sensor[1])
            if leftSensor * rightSensor != 0:
                return (leftSensor, rightSensor)

def clamp(val, small, big):
    return max(small, min(val, big))

def moveServo(angle):
    servoCorrection = 0.04
    angle = clamp(angle, -1 + servoCorrection, 1 - servoCorrection)
    angle += servoCorrection
    servo.value = angle

    
def Laps():
    global speed, servo, turnDirection, colorCube, IMAGE_WIDTH, endProgram
    
    kP = 0.08 #0.08
    kI = 0
    kD = 0.05 #0.14
    
    offsetLeft = 20
    offsetCenter = 45
    offsetRight = 70
    offsetSpeed = 0.02
    offset = offsetCenter
    currentOffset = offset
    
    leftSensor = 0
    rightSensor = 0
    
    turnDirection = 0

    time.sleep(2)
    motorStart(0)
    currentTime = time.time()
    while True:
        if endProgram == True: return 0
        
        oldTime = currentTime
        currentTime = time.time()
        deltaTime = currentTime - oldTime
                    
        (leftSensor, rightSensor) = readSensors()
        
        
        print(leftSensor, rightSensor)
        
        
        if leftSensor < 120 and rightSensor < 120:
            if ((colorCube == 'R' and turnDirection != 1) or
               (colorCube == 'G' and turnDirection == 1)):
                offset = offsetRight
            elif ((colorCube == 'R' and turnDirection == 1) or
                 (colorCube == 'G' and turnDirection != 1)):
                offset = offsetLeft
            print("offset: ", end='')
            print(offset)
            print(currentOffset)
            
            if currentOffset < offset: currentOffset += offsetSpeed
            elif currentOffset > offset: currentOffset -= offsetSpeed
            
            error = currentOffset - leftSensor   
            if turnDirection == 1: error = rightSensor - currentOffset
            
            angle = error * kP
            moveServo(angle)
            
        else: #turn
            print('turn')
            motorStart(-5)
            moveServo(0)
            time.sleeep(0.012)
            
            if turnDirection == 0:
                turnDirection = 1
                if leftSensor >= 90: turnDirection = -1
            
            print(turnDirection)
            
            motorStart(80)
            time.sleep(0.83)
            moveServo(turnDirection)
            time.sleep(1.7)
            motorStart(-5)
            moveServo(0)
            time.sleep(0.012)
            time.sle()
        
        
        
        
    ##### end of while #####
        
    motorStop()
    

if __name__ == "__main__":
    motorStop()
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    
    #button.wait_for_press()
    #print("press")
    
    moveServo(0)
    
    speed = 100
    
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
    camera.framerate = 40
    
    #try:
    rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))
    
    time.sleep(0.5)
    
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    
    endProgram = False
    
    Thread(target=arduinoReceving, args=(ser,)).start()
    ser.reset_input_buffer()
    time.sleep(2.2)
    Thread(target=Laps).start()

    time.sleep(0.1)
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        img = imgProcc(image)
        cv2.imshow("Frame", img)
        
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
    #finally:
    endProgram = True
    moveServo(0)
    motorStop()
    GPIO.cleanup()
    time.sleep(1)
    sys.exit()
    print("Program has ended")



