import wiringpi

class Motor():
    # velocity z zakresu <0;100>
    # direction clockwise->0 , counterclockwise->1 none->None or 2 (as you wish)
    def __init__(self, motorNum, velocity, direction, pwmPin):
        self.motorNum = motorNum
        self.velocity = velocity
        self.direction = direction
        self.directionCount = 0
        self.pwmPin = pwmPin
        wiringpi.softPwmCreate(self.pwmPin, 0, 100)

    def setVelocity(self, velocity):
        self.velocity = velocity
        wiringpi.softPwmWrite(self.pwmPin, self.velocity)
        print('velocity set on ', self.velocity, ' motor ', self.motorNum)

    def setDirection(self, direction):
        self.direction = direction
        if self.motorNum is 1 and self.direction is 0:
            self.directionCount = 4
        elif self.motorNum is 1 and self.direction is 1:
            self.directionCount = 8
        elif self.motorNum is 2 and self.direction is 0:
            self.directionCount = 2
        elif self.motorNum is 2 and self.direction is 1:
            self.directionCount = 16
        elif self.motorNum is 3 and self.direction is 0:
            self.directionCount = 1
        elif self.motorNum is 3 and self.direction is 1:
            self.directionCount = 64
        elif self.motorNum is 4 and self.direction is 0:
            self.directionCount = 32
        elif self.motorNum is 4 and self.direction is 1:
            self.directionCount = 128
        else:
            self.directionCount = 0

        print('direction set on ', self.directionCount, ' motor ', self.motorNum)
        return self.directionCount

    def getDirectionCount(self):
        return self.directionCount

class MotorShield():
    def __init__(self, LATCH, CLCK, SER, PWM1=None, PWM2=None, PWM3=None, PWM4=None):
        wiringpi.wiringPiSetup()
        self.LATCH = LATCH
        wiringpi.pinMode(self.LATCH, 1)
        self.CLCK = CLCK
        wiringpi.pinMode(self.CLCK, 1)
        self.SER = SER
        wiringpi.pinMode(self.SER, 1)

        self.pwmPins = []
        self.pwmPins.append(PWM1)  # index 0
        self.pwmPins.append(PWM2)
        self.pwmPins.append(PWM3)
        self.pwmPins.append(PWM4)

        self.motors = []
        for motorNum in range(0, 4):
            if self.pwmPins[motorNum] is not None:
                self.motors.append(Motor(motorNum+1, 0, None, self.pwmPins[motorNum]))
            else:
                self.motors.append(None)
        self.allDirections = 0

        if PWM1 is not None:
            self.PWM1 = PWM1
            wiringpi.pinMode(self.PWM1, 1)
        if PWM2 is not None:
            self.PWM2 = PWM2
            wiringpi.pinMode(self.PWM2, 1)
        if PWM3 is not None:
            self.PWM3 = PWM3
            wiringpi.pinMode(self.PWM3, 1)
        if PWM4 is not None:
            self.PWM4 = PWM4
            wiringpi.pinMode(self.PWM4, 1)

    def shiftWriteRegister(self, value):
		valone = value
        wiringpi.digitalWrite(self.LATCH, 0)
        for x in range(0, 8):
            temp = value & 0x80
            wiringpi.digitalWrite(self.CLCK, 0)
            if temp == 0x80:
                # data bit HIGH
                wiringpi.digitalWrite(self.SER, 1)
            else:
                # data bit LOW
                wiringpi.digitalWrite(self.SER, 0)
            wiringpi.digitalWrite(self.CLCK, 1)
            value <<= 0x01  # shift left
		
        wiringpi.digitalWrite(self.LATCH, 1)
        print('register shifted to: ', valone)

    def startMotor(self, motorNum, velocity,  direction):
        self.motors[motorNum-1].setVelocity(velocity)
        self.allDirections -= self.motors[motorNum-1].getDirectionCount()
        self.allDirections += self.motors[motorNum-1].setDirection(direction)
		
        self.shiftWriteRegister(self.allDirections)

    def stopMotor(self, motorNum):
        self.startMotor(motorNum, 0, None)