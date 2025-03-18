#include "rodos.h"
#include "topic.h"
#include "config_mpc.h"
#include "matlib.h"

struct itr_step_status
{
    long curr_iteration_step = 0;
    long curr_iteration_step_tof = 0;
    long curr_iteration_step_ekf = 0;
    long curr_iteration_step_mpc = 0;
    long last_updated_time_step = 0;
};

struct tof_to_ekf
{ 
    float tof_dist_filtered[4];
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> Xk{(float []){},"State vector X(k)"};
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> Zk_filtered{(float []){},"State vector Z(k)"};
       
};

struct ekf_to_mpc
{
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> ekf_Xk{(float []){},"State vector X(k)"};
    long curr_iteration_step = 0;
};

struct mpc_to_current_controller
{
    RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> mpc_Xk{(float []){},"State vector X(k)"};
    long curr_iteration_step = 0;
};

struct status_and_desired_current
{
    float i[4] = {0}; 
    long curr_iteration_step = 0;
    RODOS::Vector_F<EKF_TOF_SENSORS_REQ> i_dt_status{(float[]){0},{"Duty cycle input for pid current controller"}};
    long curr_iteration_step = 0;
};

extern Topic<itr_step_status> curr_itr_number_topic;
extern Topic<tof_to_ekf> tof_to_ekf_topic;
extern Topic<ekf_to_mpc> ekf_to_mpc_topic;
extern Topic<mpc_to_current_controller> mpc_to_i_topic;
extern Topic<status_and_desired_current> status_and_desired_current_topic;
