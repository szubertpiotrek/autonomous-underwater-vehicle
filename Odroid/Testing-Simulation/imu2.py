import um7
from connectionForTesting import *
import threading
import time

name1 = 's'
port1 = '/dev/ttySAC0'
statevars = ['roll', 'pitch', 'yaw']

s1 = um7.UM7(name1, port1, statevars, baud=115200)

print('GET_FW_REVISION=' +     s1.get_fw_revision())
print('ZERO_GYROS ' + 'ok.' if s1.zero_gyros()      else 'failed.')
print('RESET_EKF ' + 'ok.'  if s1.reset_ekf()       else 'failed.')
#s1.reset_to_factory()
#s1.set_mag_reference()
#s1.set_home_position()

fs = ''
hs = ''
for i in statevars:
    hs += '{:>9.9s} '
    if i == 'health':
        fs += ' {0['+i+']:08b} '
    else:
        fs += '{0['+i+']:9.3f} '

c = 0
sv = ['roll', 'pitch', 'yaw']

def readSamples(connectionObj):
    while True:
        s1.catchallsamples(sv, 1.0)
        if c % 100 == 0:
            print(hs.format(*statevars))
        print(fs.format(s1.state))
        c += 1
        connectionObj.setDataFrame(s1.state)


connThread = Connection('192.168.137.68')
threading.Thread(target=readSamples, args=[connThread]).start()
connThread.start()
