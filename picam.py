from picamera import PiCamera
import time
import datetime
camera = PiCamera()
camera.start_preview()
time.sleep(3)
camera.capture('/home/pi/Desktop/image.jpg')
camera.stop_preview()