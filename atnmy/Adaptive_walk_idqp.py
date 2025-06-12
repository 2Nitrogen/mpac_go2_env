import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R
import torch
from collections import deque

from atnmy.mpac_cmd import *
from time import sleep
from math import atan2, sin, cos, sqrt, pi
import time


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

def update_weighted_moving_average(buffer, new_value, alpha=1, threshold=0.2, conservativity=1.0):
    """
    buffer: deque of past smoothed values
    new_value: new DNN output (scalar)
    alpha: base weight for new value (default = 1.0)
    threshold: reference discrepancy scale
    conservativity: factor to suppress increasing values (0.0 ~ 1.0)

    return: new smoothed value
    """
    new_value = float(new_value)
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
    # print(f"calculated_smoothed: {smoothed}")
    return smoothed

def euler_rates_to_body_omega(roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot):
    """
    Convert Euler angle rates to body angular velocity
    """
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    ty = np.tan(pitch)

    # Transformation matrix from Euler rate to angular velocity
    T = np.array([
        [1, 0, -sp],
        [0, cr, sr * cp],
        [0, -sr, cr * cp]
    ])
    return T @ np.array([roll_dot, pitch_dot, yaw_dot])

def InBodyFrame(q, qd, foot_pos, foot_vel):

    R_gb = np.zeros( (3, 3) )
    R_gb = R.from_euler('xyz', q[3:6]).as_matrix()
    # print(f"Shape of Rot Mtx arr: {R_gb.shape}")

    # vel_body = np.zeros( 6 )
    vel_body = np.zeros( 3 )
    w_body = np.zeros( 3 )
    foot_pos_body = np.zeros( 12 )
    foot_vel_body = np.zeros( 12 )
    

    vel_body[0:3] = R_gb.T @ qd[0:3]   # Linear velocity
    # vel_body[3:6] = R_gb.T @ qd[3:6]   # Angular velocity

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
OUTPUT_BUFFER_SIZE = 500
output_buffer = deque(maxlen=OUTPUT_BUFFER_SIZE)

FILTERED_OUTPUT_BUFFER_SIZE = 200
filtered_output_buffer = deque(maxlen=FILTERED_OUTPUT_BUFFER_SIZE)
smoothed_mu_log = []
output_temp = 0
sim_time = 0



##### Modify filtering method #####

def update_filter(mu_buffer):
    mu_buffer = [m for m in mu_buffer if np.isfinite(m)]
    
    top5 = sorted(mu_buffer, reverse=True)[:3]
    if len(top5) == 0:
        top5_mean = 0.7
    else:
        top5_mean = sum( top5 ) / len( top5 )
    
    if len(mu_buffer) == 0:
        mean = 0.7
    else:
        mean = sum(mu_buffer) / len(mu_buffer)


    v = np.var( np.array(mu_buffer) )
    f = (0.6644)*( np.exp(4.1354*v)) + (-0.3764)
    w = 1.0046 / ( 1 + np.exp( (f-0.9121)*(7.6782) ) )
    # print(f"w: {w}")

    filtered_mu = (1-w)*top5_mean + w*mean
    # filtered_mu = (top5_mean + mean) / 2
    return filtered_mu

###################################



def walk_idqp_adaptive( h_cmd=0.25, vx_cmd=0, vy_cmd=0, vrz_cmd=0, mu_cmd=0.7 ):
    global state_buffer, feed_forward_counter, previous_contact, output_temp, sim_time
    ENABLE_ADAPTIVE = True              # update mu of controller by using predictions or not
    ENABLE_IMPULSE_DETECTION = True     # want to implement mu update only when impulse is detected or not
    

    # trj_col_list = list(range(9, 45))  # Joint pos/vel/torque
    trj_col_list = list(range(69, 93))   # Foot pos/vel
    cur_col_list = list(range(3, 9))     # base velocity/command velocity in body frame
    Sampling_rate = 10  # Frequency to update mu, Hz
    step_count = 0
    mu_log = []

    while (True):
        if step_count == 0:
            time_init =time.perf_counter()

        tlm_data = get_tlm_data()

        v_cmd_vec = np.array([vx_cmd, vy_cmd, vrz_cmd])
        q = tlm_data['q']
        qd = tlm_data['qd']
        tau_cmd = tlm_data['u_des']
        tau = tlm_data['u']
        q_des = tlm_data['q_des']
        foot_pos = tlm_data['feet_pos']
        # print(f"foot pos: {foot_pos}")
        foot_vel = tlm_data['feet_vel']
        # print(f"feet vel: {foot_vel}")
        vel_body, foot_pos_body, foot_vel_body = InBodyFrame( q, qd, foot_pos, foot_vel )

        # if (not np.isfinite(q).all()):
        #     print("\nq is not finite")
        # if (not np.isfinite(qd).all()):
        #     print("\nqd is not finite")
        # if (not np.isfinite(tau_cmd).all()):
        #     print("\ntau_cmd is not finite")
        # if (not np.isfinite(tau).all()):
        #     print("\ntau is not finite")
        # if (not np.isfinite(q_des).all()):
        #     print("\nq_des is not finite")
        # if (not np.isfinite(foot_pos).all()):
        #     print("\nfoot_pos is not finite")
        # if (not np.isfinite(foot_vel).all()):
        #     print("\nfoot_vel is not finite")
        # if (not np.isfinite(vel_body).all()):
        #     print("\nvel_body is not finite")


        foot_force = tlm_data['f']

        # Data processing
        rpy_scaled = Standardization( q[3:6] , np.zeros(3), np.ones(3)) 
        vel_body_scaled = Standardization(vel_body, np.zeros(3), np.ones(3)*loaded_stat['std_v_cmd'])
        v_cmd_scaled = Standardization(v_cmd_vec, np.zeros(3), np.ones(3)*loaded_stat['std_v_cmd'])
        Jnt_pos_scaled = Standardization( q[6:18], loaded_stat['standing_q'], np.ones(12)*loaded_stat['std_q'] )
        Jnt_vel_scaled = Standardization( qd[6:18], np.zeros(12), np.ones(12)*loaded_stat['std_qd'] )
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
        
        # Check if NaN data exists
        # print("New_state: ")
        # print( np.isfinite(New_state).all() )
        
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
            feed_forward_counter -= 1
            
            # for smoothing
            if output.item() < 0.05:
                output_temp = 0.05
            else:
                output_temp = output.item()
            
            output_buffer.append(output_temp)            
            filt_new = update_filter(output_buffer)
            if filt_new < 0.05:
                filt_new = 0.05

            filt_new_smoothed = update_weighted_moving_average( filtered_output_buffer, filt_new )
            New_Mu = filt_new_smoothed

            
        elif (not ENABLE_ADAPTIVE):
            New_Mu = mu_cmd

        # print( f"Predicted mu from model: {output_temp}" )
        # output_buffer.append(New_Mu)

        if (step_count % int( 1000/Sampling_rate ) == 0 ):
            if ( not np.isfinite(New_Mu) ): # Reject exception
                New_Mu = mu_cmd
            elif ( New_Mu < 0.05 ):
                New_Mu = 0.05
            print(f"Mu updated ... New_mu: {New_Mu}")
            walk_idqp( h=h_cmd, vx=vx_cmd, vy=vy_cmd, vrz=vrz_cmd, mu=New_Mu )
        
        while time.perf_counter() - time_init < sim_time:
            pass


        # Log to local file
        if step_count < 50000:
            mu_log.append( New_Mu )
        elif step_count == 50000:
            with open("/home/hjang4/Desktop/Mu_log_0612_2.pkl", "wb") as f:
                pickle.dump(mu_log, f)
                print("log saved")
        else:
            print("log already saved")

        step_count += 1
        sim_time = (0.001)*step_count
        