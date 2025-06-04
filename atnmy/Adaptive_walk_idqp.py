import os
import sys
import time
import threading
from threading import Thread
import random
import pickle
import numpy as np
import mujoco
import mujoco.viewer
from mujoco import MjModel, MjData, mj_step, mj_forward, mj_resetData
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand
import config
from scipy.spatial.transform import Rotation as R
import torch
from collections import deque

from mpac_cmd import *
from time import sleep
from math import atan2, sin, cos, sqrt, pi


def Standardization(dat, mean, std):
    dat = np.asarray(dat)
    if dat.ndim == 1:
        d = dat.shape[0]
    else:
        d = dat.shape[1]
    return (dat - mean) / std

def update_state_buffer(buffer, new_state):
    """
    buffer: (N, D) numpy array
    new_state: (D,) numpy array
    returns: updated buffer (latest data appended at end)
    """
    assert new_state.shape == (STATE_DIM,)
    buffer[:-1] = buffer[1:]         # Push up the state vectors
    buffer[-1] = new_state           # Latest state at the very last row
    return buffer

def update_weighted_moving_average(buffer, new_value, alpha=1, threshold=0.05, conservativity=0.8):
    """
    buffer: deque of past smoothed values
    new_value: new DNN output (scalar)
    alpha: base weight for new value (default = 1.0)
    threshold: reference discrepancy scale
    conservativity: factor to suppress increasing values (0.0 ~ 1.0)

    return: new smoothed value
    """
    if len(buffer) == 0:
        smoothed = new_value
    else:
        prev_avg = np.mean(buffer)
        discrepancy = new_value - prev_avg

        weight = alpha / (1 + abs(discrepancy) / threshold)

        if discrepancy > 0:
            weight *= conservativity

        smoothed = (1 - weight) * prev_avg + weight * new_value

    buffer.append(smoothed)
    return smoothed

def InBodyFrame(q, qd, foot_pos, foot_vel):

    R_gb = np.zeros( (3, 3) )
    R_gb = R.from_euler('xyz', q[:, 3:6]).as_matrix()
    # print(f"Shape of Rot Mtx arr: {R_gb.shape}")

    vel_body = np.zeros( 6 )
    w_body = np.zeros( 3 )
    foot_pos_body = np.zeros( 12 )
    foot_vel_body = np.zeros( 12 )
    

    vel_body[0:3] = R_gb.T @ qd[0:3]   # Linear velocity
    vel_body[3:6] = R_gb.T @ qd[3:6]   # Angular velocity

    w_body = euler_rates_to_body_omega( q[3], q[4], q[5], qd[3], qd[4], qd[5] )

    foot_pos_body[0:3] = R_gb.T @ ( foot_pos[0:3] - q[0:3] )
    foot_pos_body[3:6] = R_gb.T @ ( foot_pos[3:6] - q[0:3] )
    foot_pos_body[6:9] = R_gb.T @ ( foot_pos[6:9] - q[0:3] )
    foot_pos_body[9:12] = R_gb.T @ ( foot_pos[9:12] - q[0:3] )

    foot_vel_body[0:3] = R_gb.T @ ( foot_vel[0:3] - qd[0:3] - R_gb @ np.cross( np.eye(3), w_body ) @ foot_pos_body[0:3] )
    foot_vel_body[3:6] = R_gb.T @ ( foot_vel[3:6] - qd[0:3] - R_gb @ np.cross( np.eye(3), w_body ) @ foot_pos_body[3:6] )
    foot_vel_body[6:9] = R_gb.T @ ( foot_vel[6:9] - qd[0:3] - R_gb @ np.cross( np.eye(3), w_body ) @ foot_pos_body[6:9] )
    foot_vel_body[9:12] = R_gb.T @ ( foot_vel[9:12] - qd[0:3] - R_gb @ np.cross( np.eye(3), w_body ) @ foot_pos_body[9:12] )


    return vel_body, foot_pos_body, foot_vel_body

# Load model and data from Mujoco
mj_model = MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = MjData(mj_model)
mj_model.opt.timestep = config.SIMULATE_DT
mj_forward(mj_model, mj_data)
dt = mj_model.opt.timestep
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

# Load trained model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
Mymodel = torch.jit.load( "/home/hjang4/mpac_go2/trained_model_Foot_TorchScript_scaled.pt" )
Mymodel.eval()

# Load Statistic Values
stat_file_path = "/home/hjang4/Desktop/Data/STAT.pkl"  # Vx = 0.5, Vy = Vrz = 0, controller mu = 0.7, terrain mu = 0.7
with open(stat_file_path, "rb") as f:
    loaded_stat = pickle.load(f)
for name, value in loaded_stat:
    print(f"{name}: {value}")
loaded_stat = dict( loaded_stat )




BUFFER_SIZE = 200  # horizon
STATE_DIM = 93     # state vector dimension
state_buffer = np.zeros((BUFFER_SIZE, STATE_DIM ))

FEED_FORWARD_HORIZON = 120
contact_force_threshold = 5  # N (adjust if needed)
feed_forward_counter = 0
previous_contact = np.array([False, False, False, False])

# DNN output buffer for smoothing
OUTPUT_BUFFER_SIZE = 200
output_buffer = deque(maxlen=OUTPUT_BUFFER_SIZE)
smoothed_mu_log = []


def walk_idqp_adaptive( h_cmd, vx_cmd, vy_cmd, vrz_cmd, mu_cmd=0.7 ):
    global state_buffer, feed_forward_counter, previous_contact
    ENABLE_ADAPTIVE = True               # update mu of controller by using predictions or not
    ENABLE_IMPULSE_DETECTION = False     # want to implement mu update only when impulse is detected or not

    # trj_col_list = list(range(9, 45))  # Joint pos/vel/torque
    trj_col_list = list(range(69, 93))   # Foot pos/vel
    cur_col_list = list(range(3, 9))     # base velocity/command velocity in body frame
    Sampling_rate = 10  # Frequency to update mu, Hz
    step_count = 0

    while (True):
        tlm_data = get_tlm_data()

        v_cmd_vec = np.array([vx_cmd, vy_cmd, vrz_cmd])
        q = tlm_data['q']
        qd = tlm_data['qd']
        tau_cmd = tlm_data['u_des']
        tau = tlm_data['u']
        q_des = tlm_data['q_des']
        foot_pos = tlm_data['feet_pos']
        foot_vel = tlm_data['feet_vel']
        vel_body, foot_pos_body, foot_vel_body = InBodyFrame( q, qd, foot_pos, foot_vel )

        foot_force = tlm_data['f']

        # Data processing
        rpy_scaled = Standardization( q[:, 3:6] , np.zeros(3), np.ones(3)) 
        vel_body_scaled = Standardization(vel_body, np.zeros(3), np.ones(3)*loaded_stat['std_v_cmd'])
        v_cmd_scaled = Standardization(v_cmd_vec, np.zeros(3), np.ones(3)*loaded_stat['std_v_cmd'])
        Jnt_pos_scaled = Standardization( q[:, 6:18], loaded_stat['standing_q'], np.ones(12)*loaded_stat['std_q'] )
        Jnt_vel_scaled = Standardization( qd[:, 6:18], np.zeros(12), np.ones(12)*loaded_stat['std_qd'] )
        tau_cmd_scaled = Standardization( tau_cmd, loaded_stat['standing_tau_cmd'], np.ones(12)*loaded_stat['std_tau_cmd'] )
        tau_scaled = Standardization( tau, loaded_stat['standing_tau_cmd'], np.ones(12)*loaded_stat['std_tau_cmd'] )
        Jnt_pos_des_scaled = Standardization( q_des, loaded_stat['standing_q'], np.ones(12)*loaded_stat['std_q'] )
        foot_pos_body_scaled = Standardization(foot_pos_body, loaded_stat['standing_foot_pos_body'], np.ones(12)*loaded_stat['std_foot_pos_body'])
        foot_vel_body_scaled = Standardization(foot_vel_body*30, np.zeros(12), np.ones(12)*loaded_stat['std_foot_vel_body'])

        New_state = np.hstack([ rpy_scaled,                  # RPY of base, dim=3
                                vel_body_scaled,             # velocity of base in body frame, dim=3
                                v_cmd_scaled,                # command velocity of base in body frame, dim=3
                                Jnt_pos_scaled,              # joint position, dim=12
                                Jnt_vel_scaled,              # joint velocity, dim=12
                                tau_cmd_scaled,              # commanded joint torque, dim=12
                                tau_scaled,                  # measured joint torque, dim=12
                                Jnt_pos_des_scaled,          # desired joint position, dim=12
                                foot_pos_body_scaled,        # foot position in body frame, dim=12
                                foot_vel_body_scaled ])      # foot velocity in body frame, dim=12
        
        state_buffer = update_state_buffer(state_buffer, New_state)

        if (ENABLE_IMPULSE_DETECTION):
            # 1. Detect contact
            current_contact = []
            for ff in foot_force:
                contact_flag = ff > contact_force_threshold
                current_contact.append(contact_flag)
            current_contact = np.array(current_contact)

            # 2. New contact check
            new_contact = np.logical_and(current_contact, np.logical_not(previous_contact))
            if np.any(new_contact):
                feed_forward_counter = FEED_FORWARD_HORIZON

            previous_contact = current_contact.copy()
        else:
            feed_forward_counter = FEED_FORWARD_HORIZON     # always

        # 3. DNN inference only if active
        if (feed_forward_counter > 0) and (ENABLE_ADAPTIVE):
            # print("inferring..")
            input = torch.cat(( torch.tensor(state_buffer[:, trj_col_list].flatten(), dtype=torch.float32),
                                torch.tensor(state_buffer[-1, cur_col_list], dtype=torch.float32) ),
                                dim=0 )
            with torch.no_grad():
                output = Mymodel(input.to(device))
            
            # for smoothing
            if output.item() < 0.05:
                output = torch.tensor( 0.05 )
                # print(f"Raw Output: {output[0].item()}")
                output_temp = output.item()

        if (ENABLE_ADAPTIVE):
            New_Mu = update_weighted_moving_average(output_buffer, output_temp)
            if (step_count % int( 1000/Sampling_rate ) == 0 ):
                walk_idqp( h=h_cmd, vx=vx_cmd, vy=vy_cmd, vrz=vrz_cmd, mu=New_Mu )
        output_buffer.append(New_Mu)

        step_count += 1