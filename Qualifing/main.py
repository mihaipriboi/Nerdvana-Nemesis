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
servo = Servo(SERV, min_pulse_width=1.1/1000, max_pulse_width=1.9/1000, pin_factory=factory)
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
            

def readSensors():
    global ultrasonic
    
    for i in range(3):
        sensor = ultrasonic.split()
        leftSensor = int(sensor[0])
        if len(sensor) == 3:  # ca sa nu avem erori daca citeste doar un nr
            rightSensor = int(sensor[1])
            frontSensor = int(sensor[2])
            if leftSensor * rightSensor * frontSensor != 0:
                return (leftSensor, rightSensor, frontSensor)
    
    if frontSensor == 0:
        frontSensor = 400;
    return (leftSensor, rightSensor, frontSensor)

def clamp(val, small, big):
    return max(small, min(val, big))

def moveServo(angle):
    servoCorrection = -0.068
    angle = clamp(angle, -1 - servoCorrection, 1 + servoCorrection)
    angle += servoCorrection
    servo.value = -angle

def endProgram():
    moveServo(0)
    motorStop()
    GPIO.cleanup()
    time.sleep(1)
    sys.exit()
    print("Program has ended")

    
if __name__ == "__main__":
    moveServo(0)
    
    kP = 0.02 #0.08
    kI = 0
    kD = 0.08 #0.14
    speed = 100
    offset = 30
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
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.reset_input_buffer()
        Thread(target=arduinoReceving, args=(ser,)).start()
    
        time.sleep(2)
        
        ser.reset_input_buffer()
        
        time.sleep(0.1)
        
        print("Start")
        
        lastError = 0
        integral = 0
        motorStart(speed)
        isTurning = 0
        
        (leftSensor, rightSensor, frontSensor) = readSensors()
        currentOffset = rightSensor
        offsetSum = leftSensor + rightSensor
        
        start_offset = offsetSum / 2
        print(start_offset)
        currentTime = time.time()
        
        
        while turns < 11:
            (leftSensor, rightSensor, frontSensor) = readSensors()

            
            if frontSensor < 70:
                #motorStart(-100)
                #time.sleep(10)
                
                moveServo(1)
                time.sleep(0.4)
                moveServo(0)
                motorStart(-1)
                time.sleep(2)
                motorStart(speed)
                (leftSensor, rightSensor, frontSensor) = readSensors()
                currentOffset = rightSensor
                lastError = 0
                #while True:
                #    print(readSensors())
                #time.sleep(1000)
                
                
            error = currentOffset - rightSensor
            angle = kP * error + kI * integral + kD * (error - lastError)
            moveServo(clamp(angle, -1, 1))
            print(rightSensor)
            
            integral += error
            lastError = error 
        
    finally:
        endProgram()


