from w1thermsensor import W1ThermSensor
sensor = W1ThermSensor()

T_ds18b20 = sensor.get_temperature()
print("The Temperature:  %s Celcius deg" % T_ds18b20)
