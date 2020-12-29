##################
import RPi.GPIO as GPIO
###############
import time
###############
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BCM) # Use physical pin numbering

GPIO.setup(16, GPIO.OUT) #MS camera trigger
########trigger MS camera to capture mode#######
p=GPIO.PWM(16, 500)
p.start(100.0)
time.sleep(0.002)
p.stop()
GPIO.output(16, GPIO.LOW)
time.sleep(2)
########trigger MS camera to normal mode######
p=GPIO.PWM(16, 250)
p.start(100.0)
time.sleep(0.001)
p.stop()
############
GPIO.cleanup() # Clean up