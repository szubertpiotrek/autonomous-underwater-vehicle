import wiringpi
import time

wiringpi.wiringPiSetup()
wiringpi.pinMode(23, 1)
wiringpi.softPwmCreate(23, 0, 20)	# 2ms


while True:
	wiringpi.digitalWrite(23, 1)
	wiringpi.softPwmWrite(23, 15.25)
	time.sleep(2)
	wiringpi.softPwmWrite(23, 15)
	time.sleep(2)
	wiringpi.softPwmWrite(23, 14.75)
	time.sleep(2)
	
	