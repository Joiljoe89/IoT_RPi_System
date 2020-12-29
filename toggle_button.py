import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from time import sleep

def button_callback(channel):
    print("Button was pushed!")
    GPIO.output(21, GPIO.HIGH)
    sleep(1)
    GPIO.output(21, GPIO.LOW)

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BCM) # Use physical pin numbering

GPIO.setup(21, GPIO.OUT, initial= GPIO.LOW)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(24,GPIO.RISING,callback=button_callback) # Setup event on pin 10 rising edge

message = input("Press enter to quit\n\n") # Run until someone presses enter

GPIO.cleanup() # Clean up
