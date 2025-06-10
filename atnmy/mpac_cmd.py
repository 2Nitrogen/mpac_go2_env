import socket
import numpy as np
import atexit
import threading
from ctypes import *
import fcntl, array, struct
import time


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

PKT_SIZE = 3448

ctrl_mode_dtype = np.dtype([
    ('mode',       np.int32),
    ('disc_args',  np.int32,   (16,)),
    ('cont_args',  np.float64, (16,)),
], align=True)          # 200 Bytes

tlm_types = np.dtype([
    # --- 0. cycle_start_time ---
    ('start_time_sec',  np.int64),    # tv_sec
    ('start_time_nano', np.int64),    # tv_nsec

    # --- 1. Cycle & Time ---
    ('cycle_count',          np.uint64),
    ('cycle_duration',       np.float64),
    ('compute_duration',     np.float64),
    ('tictoc',               np.float64),
    ('path_compute_duration',np.float64),

    # --- 2. State & Command ---
    ('q',        np.float64, (18,)),
    ('qd',       np.float64, (18,)),
    ('qdd_sim',  np.float64, (18,)),
    ('u',        np.float64, (12,)),
    ('act_mode', np.int32,   (12,)),
    ('u_des',    np.float64, (12,)),
    ('q_des',    np.float64, (12,)),
    ('qd_des',   np.float64, (12,)),
    ('f',        np.float64, (4 ,)),
    ('temp',     np.float64, (12,)),

    # --- 3. CtrlModes ---
    ('ctrl_curr', ctrl_mode_dtype),
    ('ctrl_next', ctrl_mode_dtype),
    ('ctrl_des',  ctrl_mode_dtype),

    # --- 4. Prim-path ---
    ('prim_path_len', np.int32),
    ('prim_path',     ctrl_mode_dtype, (8,)),

    # --- 5. Feet ---
    ('feet_pos', np.float64, (12,)),
    ('feet_vel', np.float64, (12,)),
], align=True)

# print( tlm_types.itemsize )  # 3448
FIONREAD  = 0x541B

def unread_packets(sock: socket.socket) -> int:
    buf = array.array('i', [0])
    fcntl.ioctl(sock.fileno(), FIONREAD, buf, True)
    return buf[0] // PKT_SIZE


lock = threading.Lock()
tlm_data = None

def get_tlm_data():
  with lock:
    return tlm_data

def tlm_read_thread():
  global tlm_data
  packets = True
  buf = None
  while True:
    buf, addr = sock.recvfrom(PKT_SIZE) #size of tlm packet
    buf_size = len(buf)
    element_size = np.dtype(tlm_types).itemsize

    if buf_size % element_size != 0:
      raise ValueError(f"Buffer size ({buf_size}) must be a multiple of element size ({element_size}).")

    if (buf):
      with lock:
        tlm_data = np.frombuffer(buf, dtype = tlm_types)[0]


# def monitor_thread():
#   while True:
#       qlen = unread_packets(sock)
#       # if qlen:
#           # print(f"Accumulated packets: {qlen}")
#       time.sleep(0.1)


#start thread to read data
rx_thr  = threading.Thread(target=tlm_read_thread, daemon=True)
# mon_thr = threading.Thread(target=monitor_thread, daemon=True)
rx_thr.daemon = True
rx_thr.start()
# mon_thr.start()


# # Monitor tlm_data in real-time 
# while True:
#     data = get_tlm_data()
#     if data is not None:
#         cycle = int(data['cycle_count'])
#         feet_z = data['feet_pos'][2]
#         print(f"Cycle {cycle:>6} | FL_z = {feet_z:+.3f} m")
#     time.sleep(0.001)