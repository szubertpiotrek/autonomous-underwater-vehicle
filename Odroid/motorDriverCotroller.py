import wiringpi


class motorDriverController:
    def __init__(self, m1A, m1B, m2A, m2B, PWM1, PWM2):
        self.motor1A = m1A
        self.motor1B = m1B
        self.motor2A = m2A
        self.motor2B = m2B
        self.PWM1 = PWM1
        self.PWM2 = PWM2

        wiringpi.wiringPiSetup()
        wiringpi.pinMode(self.motor1A, 1)
        wiringpi.pinMode(self.motor1B, 1)
        wiringpi.pinMode(self.motor2A, 1)
        wiringpi.pinMode(self.motor2B, 1)
        wiringpi.pinMode(self.PWM1, 1)
        wiringpi.pinMode(self.PWM2, 1)
        wiringpi.softPwmCreate(self.PWM1, 0, 100)
        wiringpi.softPwmCreate(self.PWM2, 0, 100)

    def setMotorVel(self, motor, direction,  velocity):
        # velocity <0, 100>
        # dir = 1 ->forward, dir = 0 ->backward
        if 0 <= velocity <= 100:
            if motor == 1 and direction == 0:
                wiringpi.digitalWrite(self.motor1A, 0)
                wiringpi.digitalWrite(self.motor1B, 1)
                wiringpi.softPwmWrite(self.PWM1, velocity)
            elif motor == 1 and direction == 1:
                wiringpi.digitalWrite(self.motor1A, 1)
                wiringpi.digitalWrite(self.motor1B, 0)
                wiringpi.softPwmWrite(self.PWM1, velocity)
            elif motor == 2 and direction == 0:
                wiringpi.digitalWrite(self.motor2A, 1)
                wiringpi.digitalWrite(self.motor2B, 0)
                wiringpi.softPwmWrite(self.PWM2, velocity)
            elif motor == 2 and direction == 1:
                wiringpi.digitalWrite(self.motor2A, 0)
                wiringpi.digitalWrite(self.motor2B, 1)
                wiringpi.softPwmWrite(self.PWM2, velocity)
            else:
                print('Invalid motor or direction arguments! The ranges are: <0,1> ')
        else:
            print('Invalid velocity argument! Should be in range: <0,100>')
    def stopMotor(self, motor):
        self.setMotorVel(motor, 0, 0)

    def stopMotors(self):
        self.setMotorVel(1, 0, 0)
        self.setMotorVel(2, 0, 0)
