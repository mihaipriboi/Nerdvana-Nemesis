import math
import serial
import time
from gpiozero import Servo
from gpiozero import Button
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO
from threading import Thread
import sys

GPIO.setmode(GPIO.BOARD)      # Set GPIO mode to BCM
GPIO.setwarnings(False);

factory = PiGPIOFactory()

# Pin numbers
BUTTON = 26
# Motor driver
PWMA = 33
AIN1 = 29
AIN2 = 31
STBY = 36
# Servo motor
SERV = 12  # GPIO pin!!!
# Color sensor
COLOR_OUT = 19
COLOR_S2 = 21
COLOR_S3 = 23


# Setup Pins
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# Motor driver
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)
# Color sensor
GPIO.setup(COLOR_OUT,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(COLOR_S2,GPIO.OUT)
GPIO.setup(COLOR_S3,GPIO.OUT)


# Initialization
button = Button(BUTTON)
servo = Servo(SERV, min_pulse_width=1/1000, max_pulse_width=2/1000, pin_factory=factory)
# Motor driver
pwmFreq = 100
pwma = GPIO.PWM(PWMA, pwmFreq)
pwma.start(100)

#        ###############
#        # Functionuri #
#        ###############

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
    print("red value - ",red)

ultrasonic = ""
def arduinoReceving(ser):
    global ultrasonic 
    while True:
        if ser.in_waiting > 0:
            ultrasonic = ser.readline().decode('utf-8', 'ignore').rstrip()
            #print(ultrasonic)
            

def wallDiff():
    global ultrasonic, turnDirection
    
    leftSensor = 0
    rightSensor = 0
    while leftSensor * rightSensor == 0:
        sensor = ultrasonic.split()
        leftSensor = int(sensor[0])
        if len(sensor) > 1:  # ca sa nu avem erori daca citeste doar un nr
            rightSensor = int(sensor[1])
            if leftSensor * rightSensor != 0:
                #print(leftSensor, end=" - ")
                #print(rightSensor)
                #print("raw: " + ultrasonic)
                return (leftSensor, rightSensor)

def clamp(val, small, big):
    return max(small, min(val, big))

def moveServo(angle):
    servoCorrection = 0.04
    angle = clamp(angle, -1 + servoCorrection, 1 - servoCorrection)
    angle += servoCorrection
    servo.value = -angle

def firstTurn(turnDirection):
    global servo, kP, kI, kD
    moveServo(-turnDirection)
    time.sleep(1.1)
    
    startTime = time.time()
    (leftSensor, rightSensor) = wallDiff()
    offset = rightSensor
    if turnDirection == 1:
        offset = leftSensor
    integral = 0
    lastError = 0
    print(leftSensor, rightSensor, offset)
    
    while time.time() - startTime < 0.4:
        (leftSensor, rightSensor) = wallDiff()
        
        if turnDirection == 1:
            error = leftSensor - offset
        else:
            error = offset - rightSensor
        
        angle = kP * error + kI * integral + kD * (error - lastError)
        moveServo(clamp(angle, -1, 1))
    moveServo(0)
    

def endProgram():
    moveServo(0)
    motorStop()
    GPIO.cleanup()
    time.sleep(1)
    sys.exit()
    print("Program has ended")

    
if __name__ == "__main__":
    moveServo(0)
    
    kP = 0.08 #0.08
    kI = 0
    kD = 0.05 #0.14
    speed = 100
    offset = 25
    offsetSpeed = 0.003
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
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.reset_input_buffer()
        Thread(target=arduinoReceving, args=(ser,)).start()
    
        time.sleep(2)
        
        ser.reset_input_buffer()
        
        time.sleep(0.1)
        
        button.wait_for_press()
        print("Start")
        
        lastError = 0
        integral = 0
        motorStart(speed)
        isTurning = 0
        
        (leftSensor, rightSensor) = wallDiff()
        currentOffset = leftSensor
        offsetSum = leftSensor + rightSensor
        
        start_offset = offsetSum / 2
        print(start_offset)
        currentTime = time.time()
        
                
        while True:
            oldTime = currentTime
            currentTime = time.time()
            deltaTime = currentTime - oldTime
            
            (leftSensor, rightSensor) = wallDiff()
            #print(leftSensor, rightSensor)
            
            if turnDirection != 0:
                
                if turnDirection == -1:
                    error = leftSensor - currentOffset
                else:
                    error = currentOffset - rightSensor
                                 
                angle = kP * error + kI * integral + kD * (error - lastError)
                moveServo(clamp(angle, -1, 1))
                
                integral += error
                lastError = error
                
                if currentOffset < offset:
                    currentOffset += offsetSpeed
                else:
                    currentOffset -= offsetSpeed
                
                if turns > 10:
                    lastTurnTime += deltaTime
                    if lastTurnTime > 2:
                        motorStop()
                        time.sleep(60000)
                
                if abs(error) > 100:
                    if isTurning == 0:
                        isTurning = 1
                        print("Turns: ",turns)
                        turns += 1
                
                if isTurning == 1:
                    turnTime += deltaTime
                    if turnTime > 2.5:
                        turnTime = 0
                        isTurning = 0
                    
                
            if turnDirection == 0:
                if leftSensor > 100:
                    turnDirection = -1
                    firstTurn(-1)
                    (l, r) = wallDiff()
                    currentOffset = l
                elif rightSensor > 100:
                    turnDirection = 1
                    firstTurn(1)
                    (l, r) = wallDiff()
                    currentOffset = r
                else:
                    error = leftSensor - currentOffset
                    angle = kP * error + kI * integral + kD * (error - lastError)
                    moveServo(clamp(angle, -1, 1))
                    
                    integral += error
                    lastError = error
            
        motorStop()
        
        
    finally:
        endProgram()
