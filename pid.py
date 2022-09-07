import math
import serial
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO
from threading import Thread

GPIO.setmode(GPIO.BOARD)      # Set GPIO mode to BCM
GPIO.setwarnings(False);

factory = PiGPIOFactory()

# Pin numbers
BUTTON = 37
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
    in1 = GPIO.LOW
    in2 = GPIO.HIGH

    if(speed < 0):
        speed = speed * -1
        in1 = GPIO.HIGH
        in2 = GPIO.LOW

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
            ultrasonic = ser.readline().decode('utf-8').rstrip()

def wallDiff():
    global ultrasonic
    sensor = ultrasonic.split()
    leftSensor = int(sensor[0])
    rightSensor = int(sensor[1])
    if abs(leftSensor - rightSensor) < 1:
        return 0
    else:
        return leftSensor - rightSensor

def clamp(val, small, big):
    return max(small, min(val, big))

def endProgram():
    servo.value = 0;
    motorStop()
    GPIO.cleanup()
    time.sleep(1)
    print("Program has ended")

    
if __name__ == "__main__":
    NUM_CYCLES = 10
    GPIO.output(COLOR_S2,GPIO.LOW)
    GPIO.output(COLOR_S3,GPIO.LOW)
    
    servo.value = 0
    
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer;

    Thread(target=arduinoReceving, args=(ser,)).start()
    
    time.sleep(2)
    
    try:
        kP = 0.04
        kI = 0
        kD = 0
        speed = 100
        turns = 0
        
        programFlag = False
        turnDirection = 1
        
        while True:
            buttonState = GPIO.input(BUTTON)
            
            if buttonState == 0:
                lastError = 0
                integral = 0
                motorStart(80)
                cnt = 0
                  
                while buttonState == 0:
                    error = wallDiff()
                    #if error != 0:
                    #    sign  = error / abs(error)
                    #    error = abs(error) ** 0.5 * sign
    
                    angle = kP * error + kI * integral + kD * (error - lastError)
                    #print(round(angle, 3))
                    #print(round(kP * error, 3), round(kI * integral, 3), round(kD * (error - lastError), 3),"\n")
                    servo.value = clamp(angle, -1, 1)
                    
                    integral += error
                    lastError = error
                    
                    buttonState = GPIO.input(BUTTON)
                    
                    
                motorStop()
                        
            else:
                motorStop()
        
        
    finally:
        endProgram()