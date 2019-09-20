import scipy.integrate
import time


class Integrator:

    def __init__(self, sample_time=0.01):
        self.sample_time = sample_time
        self.current_time = time.time()
        self.last_time = self.current_time
        self.c = 0  # wyraz wolny z całkowania

    def integrate(self, sample):
        self.current_time = time.time()

        f = lambda x: sample
        results = scipy.integrate.quad(f, self.last_time, self.current_time)

        result = results[0] + self.c
        self.last_time = self.current_time
        self.c = result

        return result

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

