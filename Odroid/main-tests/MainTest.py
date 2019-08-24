import time
import threading
import os
from imu import IMUClass
from MotorControl import *
import DHT
from connectionForTesting import *

# it would be wise to use logging afterwards with more complicated code
# import logging

# logging.basicConfig(filename='output.log', level=logging.INFO)

IP_ADDRESS = '192.168.137.22'
from PID import PID

# horizontal left / right, vertical left/mid/right
motors_names = ['hl', 'hr', 'vl', 'vm', 'vr']   # should be connected in this order to pwm outputs 0, 1, 2,...

# list where motors' speeds will be stored
motors_speed = [0, 0, 0, 0, 0]

# list of changes to motors' speeds made by PIDs
motors_speed_diff_pid = [0, 0, 0, 0, 0]

# roll, pitch, yaw angles
RPY_angles = [0, 0, 0]


class DHTThread(threading.Thread):

   def __init__(self):
       threading.Thread.__init__(self)
       self.temp = 0
       self.humid = 0
       self.instance = DHT.DHT11(6)
   def run(self):
       while True:
           result = self.instance.read()
           if result.is_valid():
               self.humid = result.humidity
               self.temp = result.temperature
               print("Temp: {}\tHumid: {}".format(self.temp, self.humid))
           time.sleep(6)

class MotorsControlThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.motors = MotorControl(526)    # uncomment - check freqency, maybe put slightly different one

        self.motors.initialize_all()   # uncomment

    def run(self):
        while True:
            with self.lock:
                for i in range(5):

                    motors_speed[i] += motors_speed_diff_pid[i]
                    motors_speed_diff_pid[i] = 0
                    self.motors.run_motor(i, motors_speed[i])    # uncomment
                    # print("{}:{}".format(motors_names[i], motors_speed[i]), end=" ")    # comment
                # print()    # comment
                # time.sleep(3)    # comment


class IMUThread(threading.Thread):

    def __init__(self, imu):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.IMU = imu

    def run(self):
        # will start printing samples (maybe we could run it in another terminal)
        #c = 0
        while True:
            with self.lock:
                self.IMU.catchSamples()
                self.connObj.setDataFrame(self.IMU.getSamples())
                #self.IMU.printSamples(c % 50 == 0)
                #c += 1

    def getIMU(self):
        return self.IMU

    def setIMU(self, imu):
        self.IMU = imu

    def setConnection(self, connObject):
        self.connObj = connObject

class PIDThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.roll_PID = PID()
        self.pitch_PID = PID()
        self.yaw_PID = PID()
        self.IMU = None

        self.roll_diff = 0
        self.pitch_diff = 0
        self.yaw_diff = 0

        max_sum_output = 900
        self.roll_PID.setMaxOutput(max_sum_output / 3)
        self.pitch_PID.setMaxOutput(max_sum_output / 3)
        self.yaw_PID.setMaxOutput(max_sum_output / 3)

        self.pid_motors_speeds_update = [0, 0, 0, 0, 0]

    def run(self):
        while True:
            self.roll_diff = self.roll_PID.update(self.IMU.getSample('roll'))
            self.pitch_diff = self.pitch_PID.update(self.IMU.getSample('pitch'))
            self.yaw_diff = self.yaw_PID.update(self.IMU.getSample('yaw'))

        for motor in motors_speed:
            motor = 0

            self.roll_control()
            self.pitch_control()
            self.yaw_control()
            time.sleep(0.5)


    def roll_control(self):
        self.pid_motors_speeds_update [2] += self.roll_diff
        self.pid_motors_speeds_update [4] -= self.roll_diff

    def pitch_control(self):
        self.pid_motors_speeds_update [2] += self.pitch_diff / 2
        self.pid_motors_speeds_update [4] += self.pitch_diff / 2
        self.pid_motors_speeds_update [3] -= self.pitch_diff

    def yaw_control(self):
        self.pid_motors_speeds_update [0] += self.yaw_diff
        self.pid_motors_speeds_update [1] -= self.yaw_diff

    def update_motors(self):
        motors_speed_diff_pid[:] = self.pid_motors_speeds_update[:]
        self.pid_motors_speeds_update = [0] * 5

    def getIMU(self):
        return self.IMU

    def setIMU(self, imu):
        self.IMU = imu


class MotorsWaitThread(threading.Thread):

    wait_time = 0
    prev_speed = []

    def __init__(self, wait_time, prev_speed):
        threading.Thread.__init__(self)
        self.wait_time = wait_time
        self.prev_speed = prev_speed

    def run(self):
        print("Running motors for {}s".format(self.wait_time))
        time.sleep(self.wait_time)
        print("Back to: {}".format(self.prev_speed))
        motors_speed[:] = self.prev_speed[:]


class UIThread(threading.Thread):

    def __init__(self, pid_thread):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.prev_speed = motors_speed[:]
        self.motors_wait_thread = MotorsWaitThread(0, [])
        self.pid_thread = pid_thread

    def run(self):
        global motors_speed, motors_names

        while True:
            cmd = input()
            args = cmd.split(' ')
            print(args)

            with self.lock:
                if len(args) == 3:
                    self.prev_speed[:] = motors_speed[:]

                if args[0] == "s":
                    print("Stopping all motors")
                    motors_speed = [0] * 5
                    self.motors_wait_thread.prev_speed[:] = [0] * 5

                for name in motors_names:
                    if args[0] == name and args[1]:
                        motors_speed[motors_names.index(name)] = int(args[1])
                        print("Changing {} speed to {}".format(name, int(args[1])))

                if args[0] == 'h':
                    motors_speed[0] = int(args[1])
                    motors_speed[1] = int(args[1])
                    print("Changing H motors speeds to {}".format(int(args[1])))

                if args[0] == 'v':
                    motors_speed[2] = int(args[1])
                    motors_speed[3] = int(args[1])
                    motors_speed[4] = int(args[1])
                    print("Changing V motors speeds to {}".format(int(args[1])))

                if args[0] == 'vlr':
                    motors_speed[2] = int(args[1])
                    motors_speed[4] = int(args[1])
                    print("Changing V l&r motors speeds to {}".format(int(args[1])))

                if args[0] == "pid":
                    axis = args[1]
                    Kp, Ki, Kd = float(args[2]), float(args[3]), float(args[4])
                    if axis == 'x':
                        pid_thread.roll_PID.setPIDCoefficients(Kp, Ki, Kd)

                    if axis == 'y':
                        pid_thread.pitch_PID.setPIDCoefficients(Kp, Ki, Kd)

                    if axis == 'z':
                        pid_thread.yaw_PID.setPIDCoefficients(Kp, Ki, Kd)

                elif len(args) == 3:
                    motor_wait_thread = MotorsWaitThread(float(args[2]), self.prev_speed)
                    motor_wait_thread.start()


imu = IMUClass('roll', 'pitch', 'yaw')
connThread = Connection(IP_ADDRESS)
#dht_thread = DHTThread()
#motors_control_thread = MotorsControlThread()

imu_thread = IMUThread(imu)
imu_thread.setConnection(connThread)

# pid_thread = PIDThread()
#ui_thread = UIThread()


# opening another terminal and executing output.log tailing
# os.system("gnome-terminal -e 'tail -f output.log'")   # works on PC Ubuntu
# os.system("mate-terminal --window --working-directory='~/autonomous-underwater-vehicle/Odroid/main-tests' --command='tailOutput.sh'")
# os.system("sh -c '~/autonomous-underwater-vehicle/Odroid/main-tests/tailOutput.sh'")

#dht_thread.start()
#motors_control_thread.start()
imu_thread.start()
connThread.start()
# pid_thread.start()
#ui_thread.start()
