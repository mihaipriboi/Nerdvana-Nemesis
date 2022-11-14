import math
import serial
import time
from gpiozero import Servo
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO
from threading import Thread
import sys
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera


#############################################
#        Defines and Initializations        #
#############################################

################### Board ###################

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

factory = PiGPIOFactory()

################ Pin numbers ################

# Start button
BUTTON = 26

# Motor driver
PWMA = 33
AIN1 = 29
AIN2 = 31
STBY = 36

# Servo motor
SERV = 12  # GPIO pin!!!

################ Setup Pins #################

GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Motor driver
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)

############## Initialization ###############

button = Button(BUTTON)
servo = Servo(SERV, min_pulse_width=1.03/1000, max_pulse_width=1.97/1000, pin_factory=factory)

# Motor driver
pwmFreq = 100
pwma = GPIO.PWM(PWMA, pwmFreq)
pwma.start(100)

#############################################
#                 Functions                 #
#############################################

############# Image Processing ##############

cubeColor = 0
cubeX = 0
cubeY = 0
def imgProcc(frame):
    global cubeColor, cubeX, cubeY
    img = cv2.blur(frame,(3,3))
    # show the frame
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower bound and upper bound for Green color
    lower_boundGreen = np.array([50, 75, 15])  
    upper_boundGreen = np.array([95, 255, 255])
    # find the colors within the boundaries
    maskGreen = cv2.inRange(hsv, lower_boundGreen, upper_boundGreen)
    
    #red
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([5, 255, 255])
     
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([182,255,255])
     
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
    minArea = 150
    
    cubeColor = ''
    maxArea = minArea
    
    for green in contoursGreen:
        areaGreen = cv2.contourArea(green)
        if areaGreen > minArea:
            cubeColor = 'G'
            if maxArea < areaGreen:
                maxArea = areaGreen
            M = cv2.moments(green)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cubeX, cubeY = cx, cy
            #print("Coords x and y for green: ", cx, cy)
            cv2.circle(img,(cx,cy),5,(0,255,0),-1)
    for red in contoursRed:
        areaRed = cv2.contourArea(red)
        if areaRed > minArea:
            if maxArea < areaRed:
                maxArea = areaRed
                cubeColor = 'R'
            M = cv2.moments(red)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cubeX, cubeY = cx, cy
            #print("Coords x and y for red: ", cx, cy)
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
    
    return img

def getCubeColor(currentSide, turnDir):
    global cubeColor, cubeX, cubeY
    sideDist = 10
    
    cube = ''
    if currentSide == 0:
        if sideDist < cubeX and cubeX < IMAGE_WIDTH - sideDist:
            cube = cubeColor
        if 50 < cubeY:
            cube = cubeColor
    elif currentSide == -1 and turnDir == -1:
        if IMAGE_WIDTH / 2 < cubeX:
            cube = cubeColor
    elif currentSide == 1 and turnDir == 1:
        if IMAGE_WIDTH / 2 > cubeX:
            cube = cubeColor
    else:
        cube = cubeColor
        
    #print(cubeX, cubeY, cubeColor)
    return cube

################ Drive Motor ################

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

############ Arduino Connection #############

arduino_read = ""
def arduinoReceving(ser):
    global arduino_read, endProgram
    while endProgram == False:
        if ser.in_waiting > 0:
            arduino_read = ser.readline().decode('utf-8', 'ignore').rstrip()
            arduino_read = arduino_read
            print("raw: " + arduino_read) 

def readSensors():
    global arduino_read
    sensor = arduino_read.split()
    if len(sensor) > 1:
        return (int(sensor[0]), int(sensor[1]), int(sensor[2]), float(sensor[3]))
    else:
        return (0, 0, 0, 0)
    
def startSerial(tries):
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()
    time.sleep(0.1)
    ser.write(b"\n")
    time.sleep(0.1)
    if ser.in_waiting > 0:
        tries += 1
        if tries == 2:
            return ser
    
    ser.close()
    print("Serial communcation failed!")
    return startSerial(tries)

################ Servo Motor ################

def clamp(val, small, big):
    return max(small, min(val, big))

def moveServo(angle):
    servoCorrection = 0.04
    angle = clamp(angle, -1 + servoCorrection, 1 - servoCorrection)
    angle += servoCorrection
    servo.value = angle

############## Move Functions ###############
    
def sMove(startSide, finishSide):
    global servo, currentSide
    # -1 -> left; 0 -> center; 1 -> right
    if startSide != finishSide: 
        moveServo(finishSide)
        time.sleep(0.85 + abs(startSide) * 0.25)
        moveServo(-finishSide)
        time.sleep(0.88 + abs(startSide) * 0.25)
        moveServo(0)
        currentSide = finishSide

def driveToDist(dist):
    global centerAngle, turnDirection
    
    lastError = 0
    integral = 0
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
    while (leftSensor + rightSensor) < 100:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        #print(frontSensor, leftSensor, rightSensor)
        
        error = centerAngle - gyro
        angle = kP * error + kD * (error - lastError)
        lastError = error
        moveServo(angle)
        
    driveTime(1, 0.1)
    
    while frontSensor > dist:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        #print(frontSensor, leftSensor, rightSensor)
        
        error = centerAngle - gyro
        angle = kP * error + kD * (error - lastError)
        lastError = error
        moveServo(angle)
    
    if turnDirection == 0 and (leftSensor + rightSensor) > 110:
        if leftSensor > rightSensor:
            turnDirection = -1
        else:
            turnDirection = 1
        
def driveToTurn():
    driveToDist(90)

def driveToCube():
    lastError = 0
    integral = 0
    
    frontSensor = 400
    while cubeColor == '':
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        
        error = centerAngle - gyro
        angle = kP * error + kD * (error - lastError)
        lastError = error
        moveServo(angle)
        
def driveTime(direction, wait):
    lastError = 0
    integral = 0
    startTime = time.time()
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
    while time.time() - startTime < wait:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        
        error = (centerAngle - gyro) * direction
        angle = kP * error + kD * (error - lastError)
        lastError = error
        moveServo(angle)
        
def driveTimeFront(direction, wait):
    lastError = 0
    integral = 0
    startTime = time.time()
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
    while time.time() - startTime < wait:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        
        error = (centerAngle - gyro) * direction
        angle = kP * error + kD * (error - lastError)
        lastError = error
        moveServo(angle)

################### Turns ###################

def extTurn(turnDir):
    global centerAngle
    
    driveTime(1, 0.58)
    
    # Turn 90 degrees in the direction of the circuit
    centerAngle += real90Deg * turnDir
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors() 
    while (centerAngle - gyro) * turnDir > 14:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        moveServo(turnDir)
    
    moveServo(0)
    
    # Go backwards to have space for S move
    motorStop()
    time.sleep(0.5)
    motorStart(-80)
    driveTime(-1, 0.35)
    motorStop()
    time.sleep(0.5)
    
    
def centTurn(turnDir):
    global centerAngle
    
    driveTime(1, 0.55)
    
    # Turn 90 degrees in the direction of the circuit
    centerAngle += real90Deg * turnDir
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
    while (centerAngle - gyro) * turnDir > 14:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        moveServo(turnDir)
    
    moveServo(0)
    
    # Go backwards to have space for S move
    motorStop()
    time.sleep(0.5)
    motorStart(-80)
    driveTime(-1, 0.7)
    motorStop()
    time.sleep(0.5)
    
def intTurn(turnDir):
    global centerAngle
    
    # Stop near the wall
    driveToDist(15)
    time.sleep(0.03)
    motorStop()
    time.sleep(0.5)
    
    # Turn backwards, as to not hit the cube
    motorStart(-speed)
    
    centerAngle += real90Deg * turnDir
    
    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
    while (centerAngle - gyro) * turnDir > 9:
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        moveServo(-turnDir)
    
    moveServo(0)
    motorStop()
    time.sleep(0.1)
    
    motorStart(-speed)
    driveTime(-1, 0.7)
    motorStop()
    time.sleep(0.5)

################ Main Thread ################

def laps():
    global turnDirection, cubeColor, currentSide, centerAngle, endProgram
    
    # Variables
    turnDirection = 0
    currentSide = 0
    
    # Program loop start
    moveServo(0)
    time.sleep(3)
    
    
    button.wait_for_press()
    print("Run started!")
    runStartTime = time.time()
    
    (leftSensor, frontSensor, rightSensor, centerAngle) = readSensors()
    
    #centerAngle -= 0.3
    
    #while True: getCubeColor(0, turnDirection)
    
    for i in range(12):
        motorStart(speed)
        print("cube color 1: ", end='')
        print(getCubeColor(currentSide, turnDirection))
        
        if getCubeColor(currentSide, turnDirection) == 'R':
            sMove(currentSide, 1)
        elif getCubeColor(currentSide, turnDirection) == 'G':
            sMove(currentSide, -1)
            
        #driveTime(1, 0.3)
        
        driveToTurn()
        
        if currentSide * turnDirection == 1:
            intTurn(turnDirection)
        elif currentSide == 0:
            centTurn(turnDirection)
        else:
            extTurn(turnDirection)
        
        currentSide = 0
        
        motorStart(speed)
        driveToCube()
        
        print("cube color 2: ", end='')
        print(getCubeColor(currentSide, turnDirection))
        
        if getCubeColor(currentSide, turnDirection) == 'R':
            sMove(currentSide, 1)
            driveTime(1, 0.8)
        elif getCubeColor(currentSide, turnDirection) == 'G':
            sMove(currentSide, -1)
            driveTime(1, 0.8)
        
        motorStop()
        time.sleep(0.2)
        
        print("Lap " + str(i) + " done!")
        
    motorStart(speed)
    driveTime(1, 0.8)
    motorStop()
    time.sleep(1)
    
    print("Run has ended in: " + str(time.time() - runStartTime))
    
    endProgram = True

#############################################
#                   Main                    #
#############################################

    
if __name__ == "__main__":
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    
    camera = PiCamera()
    camera.resolution = (IMAGE_WIDTH, IMAGE_HEIGHT)
    camera.framerate = 40
    
    speed = 100
    
    kP = 0.038 
    kI = 0
    kD = 0.19
    offset = 0
    
    real90Deg = 84.25
    
    endProgram = False
    
    try:
        motorStop()
        moveServo(0)
        
        ser = startSerial(0)
        
        print("Serial communcation ok!")
        Thread(target=arduinoReceving, args=(ser,)).start()
        time.sleep(3.5)
        
        rawCapture = PiRGBArray(camera, size=(IMAGE_WIDTH, IMAGE_HEIGHT))
        time.sleep(1)
        Thread(target=laps).start()
        
        print("Program Start")
        
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
        
        
    finally:
        endProgram = True
        moveServo(0)
        motorStop()
        GPIO.cleanup()
        time.sleep(1)
        sys.exit()
        print("Program has ended")