import wiringpi


class submarineMotor:
	def __init__(self, pwmpin):
		self.pwmpin = pwmpin
		
		wiringpi.wiringpiSetup()
		wiringpi.pinMode(self.pwmpin, 1)
		wiringpi.softPwmCreate(self.pwmpin, 0, 20)	# 2ms
		
	def drive(self, velocity, dir):
		pass
	
	def stop(self):
		pass
		