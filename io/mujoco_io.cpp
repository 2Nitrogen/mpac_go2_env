#include "robot.h"

#include "ctrl_core.h"
#include "sim_core.h"
#include "io/mujoco_io.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

extern Robot robot;

Mujoco_io::Mujoco_io(){}

Mujoco_io::~Mujoco_io(){}

void Mujoco_io::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Mujoco_io::init() {

  InitLowCmd();

  ChannelFactory::Instance()->Init(1, "lo");

/*create publisher*/
  lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
  std::string channelName = lowcmd_publisher->GetChannelName();
  lowcmd_publisher->InitChannel();

  /*create subscriber*/
  lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
  lowstate_subscriber->InitChannel(std::bind(&Mujoco_io::LowStateMessageHandler, this, std::placeholders::_1), 1);
  
  /*loop publishing thread*/
  lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 1000, &Mujoco_io::write, this);

}

void Mujoco_io::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Mujoco_io::read(OutputVec &y) {

  Eigen::Quaterniond quat;
  quat.w() = low_state.imu_state().quaternion()[0];
  quat.x() = low_state.imu_state().quaternion()[1];
  quat.y() = low_state.imu_state().quaternion()[2];
  quat.z() = low_state.imu_state().quaternion()[3];

  Matrix3d m = quat.toRotationMatrix();
  Vector3d euler = euler_wrap(m.eulerAngles(0, 1, 2));
  
  y(S_IMU_FUSED_RX) = euler[0];
  y(S_IMU_FUSED_RY) = euler[1];
  y(S_IMU_FUSED_RZ) = euler[2];

  Vector3d gyro_global;
  gyro_global << low_state.imu_state().gyroscope()[0],
                 low_state.imu_state().gyroscope()[1],
                 low_state.imu_state().gyroscope()[2];
  Vector3d gyro_body = m*gyro_global;

  y(S_IMU_ACC_X)   = low_state.imu_state().accelerometer()[0];
  y(S_IMU_ACC_Y)   = low_state.imu_state().accelerometer()[1];
  y(S_IMU_ACC_Z)   = low_state.imu_state().accelerometer()[2];

  y(S_IMU_GYRO_RX) = gyro_body(0);
  y(S_IMU_GYRO_RY) = gyro_body(1);
  y(S_IMU_GYRO_RZ) = gyro_body(2);

  // int threshold = 15;

  y(F_FRC_FL_EE) = low_state.foot_force()[FL_];
  y(F_FRC_FR_EE) = low_state.foot_force()[FR_];
  y(F_FRC_BL_EE) = low_state.foot_force()[RL_];
  y(F_FRC_BR_EE) = low_state.foot_force()[RR_];

  // if(low_state.foot_force()[FL_] < threshold)
  //   y(F_FRC_FL_EE) = 0;

  // if(low_state.foot_force()[FR_] < threshold)
  //   y(F_FRC_FR_EE) = 0;

  // if(low_state.foot_force()[RL_] < threshold)
  //   y(F_FRC_BL_EE) = 0;

  // if(low_state.foot_force()[RR_] < threshold)
  //   y(F_FRC_BR_EE) = 0;  

  // std::cout<<"fl: "<< low_state.foot_force()[FL_] <<std::endl;
  // std::cout<<"fr: "<< low_state.foot_force()[FR_] <<std::endl;
  // std::cout<<"rl: "<< low_state.foot_force()[RL_] <<std::endl;
  // std::cout<<"rr: "<< low_state.foot_force()[RR_] <<std::endl;

  // Joints, converting from unitree convention to ours
  y(S_Q_FL1) = low_state.motor_state()[FL_0].q();
  y(S_Q_FL2) = low_state.motor_state()[FL_1].q();
  y(S_Q_FL3) = low_state.motor_state()[FL_2].q();
                                       
  y(S_Q_FR1) = low_state.motor_state()[FR_0].q();
  y(S_Q_FR2) = low_state.motor_state()[FR_1].q();
  y(S_Q_FR3) = low_state.motor_state()[FR_2].q();
                                      
  y(S_Q_BL1) = low_state.motor_state()[RL_0].q();
  y(S_Q_BL2) = low_state.motor_state()[RL_1].q();
  y(S_Q_BL3) = low_state.motor_state()[RL_2].q();
                                     
  y(S_Q_BR1) = low_state.motor_state()[RR_0].q();
  y(S_Q_BR2) = low_state.motor_state()[RR_1].q();
  y(S_Q_BR3) = low_state.motor_state()[RR_2].q();

  // std::cout<< "joint pos: " << y(S_Q_FL1) <<std::endl;
  // std::cout<< "contact force: " <<   y(F_FRC_FL_EE) <<std::endl;

  Eigen::VectorXd q_measure(12);

  for(int i(0); i<12; ++i){
    q_measure[i] = y(S_Q_FL1+i);
  }

  // std::cout<<"Measure q: "<< q_measure <<std::endl;

  y(S_QD_FL1) = low_state.motor_state()[FL_0].dq();
  y(S_QD_FL2) = low_state.motor_state()[FL_1].dq();
  y(S_QD_FL3) = low_state.motor_state()[FL_2].dq();
                                    
  y(S_QD_FR1) = low_state.motor_state()[FR_0].dq();
  y(S_QD_FR2) = low_state.motor_state()[FR_1].dq();
  y(S_QD_FR3) = low_state.motor_state()[FR_2].dq();
                                   
  y(S_QD_BL1) = low_state.motor_state()[RL_0].dq();
  y(S_QD_BL2) = low_state.motor_state()[RL_1].dq();
  y(S_QD_BL3) = low_state.motor_state()[RL_2].dq();
                                  
  y(S_QD_BR1) = low_state.motor_state()[RR_0].dq();
  y(S_QD_BR2) = low_state.motor_state()[RR_1].dq();
  y(S_QD_BR3) = low_state.motor_state()[RR_2].dq();

  y(S_U_FL1) = low_state.motor_state()[FL_0].tau_est();
  y(S_U_FL2) = low_state.motor_state()[FL_1].tau_est();
  y(S_U_FL3) = low_state.motor_state()[FL_2].tau_est();
      
  y(S_U_FR1) = low_state.motor_state()[FR_0].tau_est();
  y(S_U_FR2) = low_state.motor_state()[FR_1].tau_est();
  y(S_U_FR3) = low_state.motor_state()[FR_2].tau_est();
     
  y(S_U_BL1) = low_state.motor_state()[RL_0].tau_est();
  y(S_U_BL2) = low_state.motor_state()[RL_1].tau_est();
  y(S_U_BL3) = low_state.motor_state()[RL_2].tau_est();
    
  y(S_U_BR1) = low_state.motor_state()[RR_0].tau_est();
  y(S_U_BR2) = low_state.motor_state()[RR_1].tau_est();
  y(S_U_BR3) = low_state.motor_state()[RR_2].tau_est();

  // std::cout<<"state: "<< low_state.motor_state()[FL_0].q()<<std::endl;
  Eigen::Vector3d ground_truth;

  ground_truth[0] = low_state.imu_state().rpy()[0];
  ground_truth[1] = low_state.imu_state().rpy()[1];
  ground_truth[2] = low_state.imu_state().rpy()[2]; 

  // std::cout<<"frame_pos: "<< ground_truth <<std::endl;
  // if (mujoco_io.use_ground_truth) {
    y(S_GROUND_TRUTH_X) = ground_truth[0];
    y(S_GROUND_TRUTH_Y) = ground_truth[1];
    y(S_GROUND_TRUTH_Z) = ground_truth[2];
    y(S_GROUND_TRUTH_RX) = y(S_IMU_FUSED_RX);
    y(S_GROUND_TRUTH_RY) = y(S_IMU_FUSED_RY);
    y(S_GROUND_TRUTH_RZ) = y(S_IMU_FUSED_RZ);
  // } else {
    // y(S_GROUND_TRUTH_X) = std::numeric_limits<double>::quiet_NaN();
    // y(S_GROUND_TRUTH_Y) = std::numeric_limits<double>::quiet_NaN();
    // y(S_GROUND_TRUTH_Z) = std::numeric_limits<double>::quiet_NaN();
    // y(S_GROUND_TRUTH_RX) = std::numeric_limits<double>::quiet_NaN();
    // y(S_GROUND_TRUTH_RY) = std::numeric_limits<double>::quiet_NaN();
    // y(S_GROUND_TRUTH_RZ) = std::numeric_limits<double>::quiet_NaN();
  // }

  robot.q[0] =   ground_truth[0];
  robot.q[1] =   ground_truth[1];
  robot.q[2] =   ground_truth[2];
}

void Mujoco_io::update(const ActuatorCmds &act_update){

  act_cmds = act_update;

  // std::cout<<" motor update:"<<act_update.q[0] << std::endl;
  // std::cout<<" motor cmd:"<<act_cmds.q[0] << std::endl;

}

void Mujoco_io::write() {

  for (int i = 0; i < NUM_U; ++i) {
    if (act_cmds.mode[i] == CMD_MODE_TAU) {
      low_cmd.motor_cmd()[i].q() = PosStopF;
      low_cmd.motor_cmd()[i].dq()  = VelStopF;
      low_cmd.motor_cmd()[i].kp() = 0;
      low_cmd.motor_cmd()[i].kd() = 0;
    }
    if (act_cmds.mode[i] == CMD_MODE_TAU_VEL) {
      low_cmd.motor_cmd()[i].q() = PosStopF;
      low_cmd.motor_cmd()[i].dq() = act_cmds.qd[i];
      low_cmd.motor_cmd()[i].kp() = 0;
      low_cmd.motor_cmd()[i].kd() = act_cmds.kd[i];
    }
    if (act_cmds.mode[i] == CMD_MODE_TAU_POS) {
      low_cmd.motor_cmd()[i].q() = act_cmds.q[i];
      low_cmd.motor_cmd()[i].dq() = VelStopF;
      low_cmd.motor_cmd()[i].kp() = act_cmds.kp[i];
      low_cmd.motor_cmd()[i].kd() = 0;
    }
    if (act_cmds.mode[i] == CMD_MODE_TAU_VEL_POS) {
      low_cmd.motor_cmd()[i].q() = act_cmds.q[i];
      low_cmd.motor_cmd()[i].dq() = act_cmds.qd[i];
      low_cmd.motor_cmd()[i].kp() = act_cmds.kp[i];
      low_cmd.motor_cmd()[i].kd() = act_cmds.kd[i];
    }
    low_cmd.motor_cmd()[i].tau() = act_cmds.u[i];

  }

  // std::cout<<"command q: "<< act_cmds.q <<std::endl;
  // std::cout<<"command qd: "<< act_cmds.qd <<std::endl;  

  low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  lowcmd_publisher->Write(low_cmd);

}


void Mujoco_io::finish() {

}


