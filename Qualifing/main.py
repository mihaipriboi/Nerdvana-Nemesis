import math
import serial
import time
from gpiozero import Servo
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO
from threading import Thread
import sys

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
servo = Servo(SERV, min_pulse_width=1.1/1000, max_pulse_width=1.9/1000, pin_factory=factory)

# Motor driver
pwmFreq = 100
pwma = GPIO.PWM(PWMA, pwmFreq)
pwma.start(100)

#############################################
#                 Functions                 #
#############################################

################ Drive Motor ################

def motorStart(speed):
    speed = -speed
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
    global arduino_read
    while True:
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
    servoCorrection = 0.03
    angle = clamp(angle, -1 + servoCorrection, 1 - servoCorrection)
    angle += servoCorrection
    servo.value = angle
    
############## Move Functions ###############
    
def turnDegrees(angle):
    angleCorrection = 38
    angle = angle - angleCorrection * (angle / abs(angle))
    moveServo(-angle)
    (frontSensor, gyro) = readSensors()
    angle = gyro + angle
    while abs(angle - gyro) > 10:
        (frontSensor, gyro) = readSensors()
        
    moveServo(0)
    
#############################################
#                   Main                    #
#############################################


if __name__ == "__main__":
    moveServo(0)
    kP = 0.03
    kI = 0
    kD = 3.3 
    speed = 100
    offset = 0
    offsetSpeed = 0.005
    turnDirection = 0
    angle = 0
    turns = 0
    isTurning = 0
    firstTurnTime = 0
    firstForwardTime = 0
    turnTime = 0
    lastTurnTime = 0
    goForward = 0
    currentOffset = offset
    time_turn = 0
    
    try:
        ser = startSerial()
        
        print("Serial communcation ok!")
        Thread(target=arduinoReceving, args=(ser,)).start()
        time.sleep(8.5)
        
        print("RaspberryPi Start")
        
        lastError = 0
        integral = 0
        isTurning = 0
        leftSensor = 0
        frontSensor = 0
        rightSensor = 0
        gyro = 0.0
        center = 0
        turnStartTime = 0
        addGyro = 0
        turnDiretion = 0
        offset = 0
        flag = 0
        turnAngle = 0
        speed = 100
        
        button.wait_for_press()
        print("Run started!")
        
        motorStart(speed)
    
        
        (leftSensor, frontSensor, rightSensor, center) = readSensors()
        turnStartTime = time.time()
        waitTime = 2
        (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
        print(leftSensor, frontSensor, rightSensor, gyro)
        center = gyro

        while turns < 12:
            print(turns)
            (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
            print(leftSensor, frontSensor, rightSensor, gyro)
            
            while (leftSensor + rightSensor < 110) or time.time() - turnStartTime < waitTime:
                (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
                error = (center + offset * turnDirection) - gyro
                angle = kP * error + kD * (error - lastError)
                lastError = error
                moveServo(angle)
            
            print("Turn")
                
            if turnDirection == 0:
                (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
                while leftSensor + rightSensor < 100:
                    (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
                if rightSensor > leftSensor:
                    turnDirection = 1
                else:
                    turnDirection = -1
                
            center += 84.1 * turnDirection
            
            turns += 1
            turnStartTime = time.time()
            waitTime = 2.3
        
        waitTime = 1.8
        turnStartTime = time.time()
        while time.time() - turnStartTime < waitTime:
            (leftSensor, frontSensor, rightSensor, gyro) = readSensors()
            error = (center + offset * turnDirection) - gyro
            angle = kP * error + kD * (error - lastError)
            lastError = error
            moveServo(angle)
        
        # Last turn
        
        moveServo(0)
        
        motorStart(-1)
        time.sleep(1)
        motorStop()
        time.sleep(100)
        
    finally:
        moveServo(0)
        motorStop()
        GPIO.cleanup()
        time.sleep(1)
        sys.exit()
        print("Program has ended")