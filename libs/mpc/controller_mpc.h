#include "rodos.h"
#include "matlib.h"
#include "config_mpc.h"
#include "topic_config.h"
#include "filter_ekf.h"
#include <math.h>

typedef enum 
{
   ALLIGNMENT ,
   APPROACH ,
   DOCKING
}allignment_phase;

class mpc_input : public Thread
{

private :

    double N;    
    static double dt;
    static double curr_itr_step;
    
    static RODOS::Vector_F<MPC_CONTROL_INPUT_VECTOR_SIZE> stsp_U{(float[]){},"State Space Control Input Vector U"};
    static RODOS::Vector_F<MPC_CONTROL_INPUT_VECTOR_SIZE> stsp_U_t{(float[]){},"State Space Control Input Vector transpose Ut"};
    static RODOS::Matrix_F<MPC_PREDICTION_HORIZON,MPC_PREDICTION_HORIZON> stsp_A{(float[]){1,EKF_TOF_SAMP_TIME_DT,0,0,0,1,0,0,0,0,1,EKF_TOF_SAMP_TIME_DT,0,0,0,1},"State Transiiton Matrix A"};
    static RODOS::Vector_F<MPC_STATE_SPACE_SIZE> X_k{(float[]){},"State Vector Xk"};
    static RODOS::Vector_F<MPC_STATE_SPACE_SIZE> F{(float[]){},"State diffrence Matrix F"};
    static RODOS::Vector_F<MPC_STATE_SPACE_SIZE> F_t{(float[]){},"State diffrence Matrix traspose Ft"};
    static RODOS::Matrix_F<MPC_PREDICTION_HORIZON,MPC_CONTROL_INPUT_SIZE> stsp_B{(float[]){0,0,(EKF_TOF_SAMP_TIME_DT/TAMARIW_SATELLITE_MASS),0,0,0,0,(EKF_TOF_SAMP_TIME_DT/TAMARIW_SATELLITE_MOI)},"State Control Input Transition Matrix B"};
    // static RODOS::Vector_F<MPC_CONTROL_INPUT_SIZE> stsp_U{(float[]){0,0},"State Control Input Transition Matrix B"};
    static RODOS::Matrix_F<MPC_STATE_SPACE_SIZE,MPC_STATE_SPACE_SIZE> costfn_Q{(float[]){},"SE Cost Function Matrix Q"};
    static RODOS::Matrix_F<2,2> costfn_R{(float[]){0.1,0,0,0.1},"CE Cost Function Matrix R"};
    static RODOS::Vector_F<MPC_STATE_SPACE_SIZE> X_des{(float[]){},{"X_desired"}};
    static RODOS::Vector_F<MPC_STATE_SPACE_SIZE> X_curr{(float[]){},{"X_current"}};
    static allignment_phase curr_phase = ALLIGNMENT;
    static double J = 0;
    const float x_tolerance = 

public : 

    mpc_input(const char* t_name , const int priority , const int stack_size , double pred_horz = MPC_PREDICTION_HORIZON) : 
             Thread(t_name,priority,stack_size) , N(pred_horz) {}    

    void mpc_phase_selector()

    void init_matrix();

    void state_space_init();

    void alignment_phase();

    void approach_phase();
    
    void Docking_phase();
    
    void phase_switch_case();

    void publish_mpc_result();
   
    void init();

    void run();
};
