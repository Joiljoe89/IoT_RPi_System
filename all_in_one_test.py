# Integrated program for controlling Picamera, gy30 light intensity, bme280, ds18b20 sensors# Joe Johnson D17024, PD, IoT lab, IIT Mandi
##################
import RPi.GPIO as GPIO

###############
import datetime
import time
###############
import smbus
#######gy30####
# Define some constants from the datasheet

DEVICE     = 0x23 # Default device I2C address

POWER_DOWN = 0x00 # No active state
POWER_ON   = 0x01 # Power on
RESET      = 0x07 # Reset data register value

# Start measurement at 4lx resolution. Time typically 16ms.
CONTINUOUS_LOW_RES_MODE = 0x13
# Start measurement at 1lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_1 = 0x10
# Start measurement at 0.5lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_1 = 0x20
# Start measurement at 0.5lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_2 = 0x21
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_LOW_RES_MODE = 0x23

#bus = smbus.SMBus(0) # Rev 1 Pi uses 0
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1

def convertToNumber(data):
  # Simple function to convert 2 bytes of data
  # into a decimal number. Optional parameter 'decimals'
  # will round to specified number of decimal places.
  result=(data[1] + (256 * data[0])) / 1.2
  return (result)

def readLight(addr=DEVICE):
  # Read data from I2C interface
  data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
  return convertToNumber(data)
########bme280#######
# Get I2C bus
bus = smbus.SMBus(1)
###############
from picamera import PiCamera
camera = PiCamera()
########ds18b20#######
import os                                                  # import os module
import glob                                                # import glob module                                                # import time module
os.system('modprobe w1-gpio')                              # load one wire communication device kernel modules
os.system('modprobe w1-therm')                                                 
base_dir = '/sys/bus/w1/devices/'                          # point to the address
device_folder = glob.glob(base_dir + '28*')[0]             # find device with address starting from 28*
device_file = device_folder + '/w1_slave'                  # store the details
def read_temp_raw():
   f = open(device_file, 'r')
   lines = f.readlines()                                   # read the device details
   f.close()
   return lines
################
a = 1
b = 2
###############
while a < b:
    time.sleep(5)
    ####gy30####
    lightLevel = readLight()
    lux = int(lightLevel)
    
    #########BME280#######
    # Get I2C bus
    bus = smbus.SMBus(1)

    b1 = bus.read_i2c_block_data(0x77, 0x88, 24)

    # Convert the data
    # Temp coefficents
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536

    # Pressure coefficents
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536

    # BME280 address, 0x76(118)
    # Read data back from 0xA1(161), 1 byte
    dig_H1 = bus.read_byte_data(0x77, 0xA1)

    # BME280 address, 0x76(118)
    # Read data back from 0xE1(225), 7 bytes
    b1 = bus.read_i2c_block_data(0x77, 0xE1, 7)

    # Convert the data
    # Humidity coefficents
    dig_H2 = b1[1] * 256 + b1[0]
    if dig_H2 > 32767 :
        dig_H2 -= 65536
    dig_H3 = (b1[2] &  0xFF)
    dig_H4 = (b1[3] * 16) + (b1[4] & 0xF)
    if dig_H4 > 32767 :
        dig_H4 -= 65536
    dig_H5 = (b1[4] / 16) + (b1[5] * 16)
    if dig_H5 > 32767 :
        dig_H5 -= 65536
    dig_H6 = b1[6]
    if dig_H6 > 127 :
        dig_H6 -= 256

    # BME280 address, 0x76(118)
    # Select control humidity register, 0xF2(242)
    #       0x01(01)    Humidity Oversampling = 1
    bus.write_byte_data(0x77, 0xF2, 0x01)
    # BME280 address, 0x76(118)
    # Select Control measurement register, 0xF4(244)
    #       0x27(39)    Pressure and Temperature Oversampling rate = 1
    #                   Normal mode
    bus.write_byte_data(0x77, 0xF4, 0x27)
    # BME280 address, 0x76(118)
    # Select Configuration register, 0xF5(245)
    #       0xA0(00)    Stand_by time = 1000 ms
    bus.write_byte_data(0x77, 0xF5, 0xA0)

    time.sleep(0.5)

    # BME280 address, 0x76(118)
    # Read data back from 0xF7(247), 8 bytes
    # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
    # Temperature xLSB, Humidity MSB, Humidity LSB
    data = bus.read_i2c_block_data(0x77, 0xF7, 8)

    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

    # Convert the humidity data
    adc_h = data[6] * 256 + data[7]

    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
    var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0
    fTemp = cTemp * 1.8 + 32

    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_P6) / 32768.0
    var2 = var2 + var1 * (dig_P5) * 2.0
    var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
    var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_P1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_P9) * p * p / 2147483648.0
    var2 = p * (dig_P8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100

    # Humidity offset calculations
    var_H = ((t_fine) - 76800.0)
    var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)))
    humidity = var_H * (1.0 -  dig_H1 * var_H / 524288.0)
    if humidity > 100.0 :
        humidity = 100.0
    elif humidity < 0.0 :
        humidity = 0.0

    # Output data to screen
    print "Temperature in Celsius : %.2f C" %cTemp
    #print "Temperature in Fahrenheit : %.2f F" %fTemp
    print "Pressure : %.2f hPa " %pressure
    print "Relative Humidity : %.2f %%" %humidity
    ###########ds18b20##############
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':                   # ignore first line
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')                        # find temperature in the details
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0                 # convert to Celsius
        #temp_f = temp_c * 9.0 / 5.0 + 32.0                   # convert to Fahrenheit 
    print "Temperature in Celsius : %.2f C" %temp_c
    
    file1 = open("/home/pi/Desktop/ss_north_field/ss_north_field_env_data.txt","a")
    #L = ["This is Delhi \n","This is Paris \n","This is Thrissur \n"]
    dt_date = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    #print(dt_date)
    file1.write(dt_date+", Intensity: %d, DS18B20_Temp: %.2f, BME280_Temp: %.2f, BME280_Pressure: %.2f,BME280_Humidity: %.2f  \n" %(lux,temp_c,cTemp,pressure,humidity)) 
    #file1.writelines(L)
    file1.close() 
    
    ###########
    print("lux",lux)
    thr = 50
    while lux > thr:
        #######GY30###########
        lightLevel = readLight()
        lux = int(lightLevel)
        print("lux",lux)
        #########BME280#######
        # Get I2C bus
        bus = smbus.SMBus(1)

        b1 = bus.read_i2c_block_data(0x77, 0x88, 24)

        # Convert the data
        # Temp coefficents
        dig_T1 = b1[1] * 256 + b1[0]
        dig_T2 = b1[3] * 256 + b1[2]
        if dig_T2 > 32767 :
            dig_T2 -= 65536
        dig_T3 = b1[5] * 256 + b1[4]
        if dig_T3 > 32767 :
            dig_T3 -= 65536

        # Pressure coefficents
        dig_P1 = b1[7] * 256 + b1[6]
        dig_P2 = b1[9] * 256 + b1[8]
        if dig_P2 > 32767 :
            dig_P2 -= 65536
        dig_P3 = b1[11] * 256 + b1[10]
        if dig_P3 > 32767 :
            dig_P3 -= 65536
        dig_P4 = b1[13] * 256 + b1[12]
        if dig_P4 > 32767 :
            dig_P4 -= 65536
        dig_P5 = b1[15] * 256 + b1[14]
        if dig_P5 > 32767 :
            dig_P5 -= 65536
        dig_P6 = b1[17] * 256 + b1[16]
        if dig_P6 > 32767 :
            dig_P6 -= 65536
        dig_P7 = b1[19] * 256 + b1[18]
        if dig_P7 > 32767 :
            dig_P7 -= 65536
        dig_P8 = b1[21] * 256 + b1[20]
        if dig_P8 > 32767 :
            dig_P8 -= 65536
        dig_P9 = b1[23] * 256 + b1[22]
        if dig_P9 > 32767 :
            dig_P9 -= 65536

        # BME280 address, 0x76(118)
        # Read data back from 0xA1(161), 1 byte
        dig_H1 = bus.read_byte_data(0x77, 0xA1)

        # BME280 address, 0x76(118)
        # Read data back from 0xE1(225), 7 bytes
        b1 = bus.read_i2c_block_data(0x77, 0xE1, 7)

        # Convert the data
        # Humidity coefficents
        dig_H2 = b1[1] * 256 + b1[0]
        if dig_H2 > 32767 :
            dig_H2 -= 65536
        dig_H3 = (b1[2] &  0xFF)
        dig_H4 = (b1[3] * 16) + (b1[4] & 0xF)
        if dig_H4 > 32767 :
            dig_H4 -= 65536
        dig_H5 = (b1[4] / 16) + (b1[5] * 16)
        if dig_H5 > 32767 :
            dig_H5 -= 65536
        dig_H6 = b1[6]
        if dig_H6 > 127 :
            dig_H6 -= 256

        # BME280 address, 0x76(118)
        # Select control humidity register, 0xF2(242)
        #       0x01(01)    Humidity Oversampling = 1
        bus.write_byte_data(0x77, 0xF2, 0x01)
        # BME280 address, 0x76(118)
        # Select Control measurement register, 0xF4(244)
        #       0x27(39)    Pressure and Temperature Oversampling rate = 1
        #                   Normal mode
        bus.write_byte_data(0x77, 0xF4, 0x27)
        # BME280 address, 0x76(118)
        # Select Configuration register, 0xF5(245)
        #       0xA0(00)    Stand_by time = 1000 ms
        bus.write_byte_data(0x77, 0xF5, 0xA0)

        time.sleep(0.5)

        # BME280 address, 0x76(118)
        # Read data back from 0xF7(247), 8 bytes
        # Pressure MSB, Pressure LSB, Pressure xLSB, Temperature MSB, Temperature LSB
        # Temperature xLSB, Humidity MSB, Humidity LSB
        data = bus.read_i2c_block_data(0x77, 0xF7, 8)

        # Convert pressure and temperature data to 19-bits
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

        # Convert the humidity data
        adc_h = data[6] * 256 + data[7]

        # Temperature offset calculations
        var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
        t_fine = (var1 + var2)
        cTemp = (var1 + var2) / 5120.0
        fTemp = cTemp * 1.8 + 32

        # Pressure offset calculations
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (dig_P6) / 32768.0
        var2 = var2 + var1 * (dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9) * p * p / 2147483648.0
        var2 = p * (dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100

        # Humidity offset calculations
        var_H = ((t_fine) - 76800.0)
        var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)))
        humidity = var_H * (1.0 -  dig_H1 * var_H / 524288.0)
        if humidity > 100.0 :
            humidity = 100.0
        elif humidity < 0.0 :
            humidity = 0.0

        # Output data to screen
        print "Temperature in Celsius : %.2f C" %cTemp
        print "Temperature in Fahrenheit : %.2f F" %fTemp
        print "Pressure : %.2f hPa " %pressure
        print "Relative Humidity : %.2f %%" %humidity
        ###########ds18b20##############
        lines = read_temp_raw()
        while lines[0].strip()[-3:] != 'YES':                   # ignore first line
            time.sleep(0.2)
            lines = read_temp_raw()
        equals_pos = lines[1].find('t=')                        # find temperature in the details
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0                 # convert to Celsius
            #temp_f = temp_c * 9.0 / 5.0 + 32.0                   # convert to Fahrenheit 
        print "Temperature in Celsius : %.2f C" %temp_c
        ######save######
        file1 = open("/home/pi/Desktop/ss_north_field/ss_north_field_env_data.txt","a")
        #L = ["This is Delhi \n","This is Paris \n","This is Thrissur \n"]
        dt_date = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        #print(dt_date)
        file1.write(dt_date+", Intensity: %d, DS18B20_Temp: %.2f, BME280_Temp: %.2f, BME280_Pressure: %.2f,BME280_Humidity: %.2f  \n" %(lux,temp_c,cTemp,pressure,humidity)) 
        #file1.writelines(L)
        file1.close()
        
        #######Picamera#########
        #GPIO Basic initialization
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        #Use a variable for the  Pin to use
        #If you followed my pictures, it's port 7 => BCM 4
        led = 18
        #Initialize your pin
        GPIO.setup(led,GPIO.OUT)
        GPIO.output(led,0)
        #time.sleep(10)
        camera.start_preview()
        time.sleep(5)
        camera.capture('/home/pi/Desktop/ss_north_field/images/image%s.jpg' %dt_date)
        camera.stop_preview()
        GPIO.output(led,1)
        time.sleep(15)
        #############################end##########





# Integrated program for controlling Picamera, gy30 light intensity, bme280, ds18b20 sensors# Joe Johnson D17024, PD, IoT lab, IIT Mandi