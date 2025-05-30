import socket
import numpy as np
import atexit
import threading
from ctypes import *

class buff(Structure):
    _fields_ = [('mode', c_int),
                ('pad', c_int),
                ('disc', c_int * 16),
                ('cont', c_double * 16)]

UDP_IP = "127.0.0.1"
CTRL_UDP_PORT = 8081
ATNMY_UDP_PORT = 8082

print("UDP target IP:", UDP_IP)
print("UDP target port:", CTRL_UDP_PORT)
print("UDP receive port:", ATNMY_UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
  sock.bind((UDP_IP, ATNMY_UDP_PORT))
  # sock.bind(("0.0.0.0", ATNMY_UDP_PORT))
except Exception as e:
  print(f"Binding Failed {e}")

def hard_stop():
  x = buff()
  x.mode = 0
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))

def soft_stop():
  x = buff()
  x.mode = 1
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))

def lie():
  x = buff()
  x.mode = 2
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def stand_idqp(h=0.25, rx=0, ry=0, rz=0):
  # print("stand")
  x = buff()
  x.mode = 5
  x.cont[0] = h
  x.cont[1] = rx 
  x.cont[2] = ry 
  x.cont[3] = rz 
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def walk_pd(gait=0):
  x = buff()
  x.mode = 7
  x.disc[0] = gait
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def walk_idqp(h=0.25, vx=0, vy=0, vrz=0, mu=0.7, t_step=0.25, h_step=0.075, t_dwell=0.05):
  x = buff()
  x.mode = 8
  x.cont[0] = h
  x.cont[1] = vx 
  x.cont[2] = vy 
  x.cont[3] = vrz
  x.cont[4] = mu
  x.cont[5] = t_step
  x.cont[6] = h_step
  x.cont[7] = t_dwell
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def walk_quasi_idqp(h=0.25, vx=0, vy=0, vrz=0):
  x = buff()
  x.mode = 9
  x.cont[0] = h
  x.cont[1] = vx 
  x.cont[2] = vy 
  x.cont[3] = vrz 
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def walk_quasi_planned(h=0.22, vx=0, vy=0, vrz=0, x=0, y=0, rz=0, mode="vel"):
  b = buff()
  b.mode = 10
  b.disc[0] = 1 if mode=="pos" else 0
  b.cont[0] = h
  b.cont[1] = vx 
  b.cont[2] = vy 
  b.cont[3] = vrz 
  b.cont[4] = x 
  b.cont[5] = y 
  b.cont[6] = rz 
  sock.sendto(bytes(b), (UDP_IP, CTRL_UDP_PORT))
   
def bound(vx=0):
  x = buff()
  x.mode =11 
  x.cont[0] = vx
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def jump(x_vel=0, y_vel=0,z_vel=2):
  x = buff()
  x.mode = 12 
  x.cont[0] = x_vel
  x.cont[1] = y_vel
  x.cont[2] = z_vel
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def land():
  x = buff()
  x.mode = 13 
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def traj_track(traj_num = 0):
  x = buff()
  x.mode = 6
  x.disc[0] = traj_num
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
def calibrate(traj_num = 0):
  x = buff()
  x.mode = 14
  x.disc[0] = traj_num
  sock.sendto(bytes(x), (UDP_IP, CTRL_UDP_PORT))
   
#run soft_stop when script is exited
atexit.register(soft_stop)

#order for q is: x, y, z,
#                rx, ry, rz,
#                fl1, fl2, fl3,
#                fr1, fr2, fr3,
#                bl1, bl2, bl3,
#                br1, br2, br3
#order for u is: fl1, fl2, fl3,
#                fr1, fr2, fr3,
#                bl1, bl2, bl3,
#                br1, br2, br3
tlm_types = np.dtype([('start_time_sec', np.int64),
                      ('start_time_nano', np.int64),
                      ('cycle_count', np.uint64),
                      ('cycle_duration', np.double),
                      ('compute_duration', np.double),
                      ('tictoc', np.double),
                      ('path_compute_duration', np.double),
                      ('q', np.double, (18,)),
                      ('qd', np.double, (18,)),
                      ('qdd_sim', np.double, (18,)),
                      ('u', np.double, (12,)),
                      ('act_mode', np.int32, (12,)),
                      ('u_des', np.double, (12,)),
                      ('q_des', np.double, (12,)),
                      ('qd_des', np.double, (12,)),
                      ('f', np.double, (4,)),
                      ('temp', np.double, (12,)),
                      ('ctrl_curr', np.int32, (1,)),
                      ('ctrl_curr_disc_args', np.int32, (16,)),
                      ('ctrl_curr_cont_args', np.double, (16,)),
                      ('ctrl_next', np.int32, (1,)),
                      ('ctrl_next_disc_args', np.int32, (16,)),
                      ('ctrl_next_cont_args', np.double, (16,)),
                      ('ctrl_des', np.int32, (1,)),
                      ('ctrl_des_disc_args', np.int32, (16,)),
                      ('ctrl_des_cont_args', np.double, (16,)),
                      ('prim_path_len', np.int32, (1,)),
                      ('prim_path_1', np.int32, (1,)),
                      ('prim_path_1_disc_args', np.int32, (16,)),
                      ('prim_path_1_cont_args', np.double, (16,)),
                      ('prim_path_2', np.int32, (1,)),
                      ('prim_path_2_disc_args', np.int32, (16,)),
                      ('prim_path_2_cont_args', np.double, (16,)),
                      ('prim_path_3', np.int32, (1,)),
                      ('prim_path_3_disc_args', np.int32, (16,)),
                      ('prim_path_3_cont_args', np.double, (16,)),
                      ('prim_path_4', np.int32, (1,)),
                      ('prim_path_4_disc_args', np.int32, (16,)),
                      ('prim_path_4_cont_args', np.double, (16,)),
                      ('prim_path_5', np.int32, (1,)),
                      ('prim_path_5_disc_args', np.int32, (16,)),
                      ('prim_path_5_cont_args', np.double, (16,)),
                      ('prim_path_6', np.int32, (1,)),
                      ('prim_path_6_disc_args', np.int32, (16,)),
                      ('prim_path_6_cont_args', np.double, (16,)),
                      ('prim_path_7', np.int32, (1,)),
                      ('prim_path_7_disc_args', np.int32, (16,)),
                      ('prim_path_7_cont_args', np.double, (16,)),
                      ('prim_path_8', np.int32, (1,)),
                      ('prim_path_8_disc_args', np.int32, (16,)),
                      ('prim_path_8_cont_args', np.double, (16,)),
                     ], align=True)
lock = threading.Lock()
tlm_data = None

def get_tlm_data():
  return tlm_data

def tlm_read_thread():
  global tlm_data
  packets = True
  buf = None
  while True:
    buf, addr = sock.recvfrom(3248) #size of tlm packet
    buf_size = len(buf)
    element_size = np.dtype(tlm_types).itemsize

    if buf_size % element_size != 0:
      raise ValueError(f"Buffer size ({buf_size}) must be a multiple of element size ({element_size}).")

    if (buf):
      with lock:
        tlm_data = np.frombuffer(buf, dtype = tlm_types)[0]

#start thread to read data
t = threading.Thread(target=tlm_read_thread)
t.daemon = True
t.start()
