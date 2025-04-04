#ifndef TOPIC_CONFIG_H
#define TOPIC_CONFIG_H

#include "rodos.h"
#include "topic.h"
#include "topics.h"
#include "config_mpc.h"
#include "matlib.h"



typedef enum 
{
   ALLIGNMENT ,
   APPROACH ,
   DOCKING
}allignment_phase;

struct itr_step_status
{
    long curr_iteration_step;
    long curr_iteration_step_tof;
    long curr_iteration_step_ekf;
    long curr_iteration_step_mpc;
    long last_updated_time_step;
};

struct tof_to_ekf
{ 
    float dt;                   // Thread period, millis
    float dm;                   // Winsorized mean of distances, mm
    float vm;                   // Winsorized mean of velocity, mm/s
    float roll;                 // Roll calculation
    float w_roll;               // Angular velocity roll
    allignment_phase mpc_phase;
};

struct ekf_buff
{
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> Xk{(float []){},"State vector X(k)"};                       
    RODOS::Matrix_F<EKF_STATE_TRANSITION_MATRIX_SIZE> P{(float []){1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},"Predicted State Covariance Matrix_F P(k)(Default to EYE(0,1))"};
    RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> Kk{(float []){},"Kalman Gain K(k)"};
    RODOS::Vector_F<EKF_CONTROL_INPUT_VECTOR> PM_Uk{(float []){},"Control Input vector U(k)"};
};

struct ekf_to_mpc
{
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> ekf_Xk{(float []){},"State vector X(k)"};
    allignment_phase mpc_phase;
};

struct mpc_buff
{
    RODOS::Matrix_F<2,1> buff_F{(float[]){0},"State diffrence Matrix F"};
    allignment_phase mpc_phase;
};

struct status_and_desired_current
{
    float i_dt_status[4] = {0,0,0,0};
    double desired_current;
};

extern Topic<itr_step_status> curr_itr_number_topic;
extern Topic<tof_to_ekf> tof_to_ekf_topic;
extern Topic<ekf_to_mpc> ekf_to_mpc_topic;
extern Topic<mpc_buff> mpc_last_ctrlinput_buff_topic;
extern Topic<status_and_desired_current> mpc_to_i_topic;
extern Topic<ekf_buff> ekf_last_state_buff_topic;
 

extern CommBuffer<tof_to_ekf> ekf_rx;
extern Subscriber subs_ekf;

extern CommBuffer<itr_step_status> ekf_itr_status;
extern Subscriber subs_ekf_itr;

extern CommBuffer<ekf_to_mpc> mpc_rx;
extern Subscriber subs_mpc;

extern CommBuffer<status_and_desired_current> magctrl_rx;
extern Subscriber subs_magctrl;

extern CommBuffer<ekf_buff> ekf_last_state_buff;
extern Subscriber subs_ekf_buff;

extern CommBuffer<mpc_buff> mpc_ctrlinput_buff;
extern Subscriber subs_mpc_buff;

#endif // TOPIC_CONFIG_H