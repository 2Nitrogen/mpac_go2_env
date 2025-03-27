import numpy as np
import h5py
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as p

p.rcParams.update({
  "text.usetex": True,
  "font.size": 22})

file = h5py.File("data/latest.h5",'r')
data = file['time_series']

#qd = data['qd'][7500:9500,6:-1]
qd = data['qd'][:,6:-1]

#N=10
#for i in range(len(qd[0])):
#  mov_avg = np.convolve(qd[:,i], np.ones((N,))/N, mode='valid')

p.plot(data['epoch_time'][:] - data['epoch_time'][0],data['qd'][:])
p.grid()
p.axhline(y=1, color='r', linestyle='--')
p.axhline(y=-1, color='r', linestyle='--')
#p.ylim(-1.9,2.6)
#p.xlim(-170,3850)
#p.yticks([-1,0,1,2])
p.title('Velocity')
p.xlabel('time (ms)')
p.ylabel('$\dot{q}$ (rad/s)')

p.show()
