import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)

p=GPIO.PWM(16, 250)
p.start(100.0)
time.sleep(0.001)
p.stop()


GPIO.cleanup()