# code for PWM implementation on Raspberry Pi

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

led = 12
trigger = 14
echo = 23

maxDutyCycle = 100 #max duty Cycle of my LED.
maxDist = 0.5 #max distance to detect in metres

GPIO.setup(led,GPIO.OUT)
pwm = GPIO.PWM(led, maxDutyCycle)
pwm.start(0)

def brightness(distance):
    if (distance > maxDist):
        pwm.ChangeDutyCycle(0)
    else:
        dutyCycle = ((maxDist-distance)/maxDist)*maxDutyCycle
        pwm.ChangeDutyCycle(dutyCycle)


def loop():
    while True:
        
        GPIO.setup(trigger,GPIO.OUT)
        GPIO.setup(echo,GPIO.IN)
        
        start = 0
        finish = 0
        #send the trigger ultrasound waves to the surrounds
        GPIO.output(trigger, GPIO.LOW)
        time.sleep(0.01)  

        GPIO.output(trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trigger, GPIO.LOW)

        #calculate time it takes for trigger to echo back to the sensor
        while (GPIO.input(echo) == GPIO.LOW):
            start = time.time()
        
        while (GPIO.input(echo) == GPIO.HIGH):
            finish = time.time()

        distance  = ((finish - start) * 343) /2  #Distance = (Time x Speed of Sound)/2
        brightness(distance)
try:
    loop()
finally:
    pwm.stop()
    GPIO.cleanup()
