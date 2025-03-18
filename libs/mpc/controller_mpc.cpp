#include "controller_mpc.h"

tof_to_ekf mpc_phase_selector_temp;

ekf_to_mpc mpc_rx_temp;
mpc_to_current_controller mpc_tx_temp;

void mpc_input::mpc_phase_selector()
{
    ekf_rx.getOnlyIfNewData(mpc_phase_selector_temp);
    if((abs(mpc_phase_selector_temp.tof_dist_filtered[2]) > MPC_THETA_TOLERANCE_VAL)
       ||(abs(mpc_phase_selector_temp.tof_dist_filtered[3]) > MPC_THETA_TOLERANCE_VAL))
    { curr_phase = ALLIGNMENT; }

    else if(if((abs(mpc_phase_selector_temp.tof_dist_filtered[0]) > MPC_X_TOLERANCE_VAL)
               ||(abs(mpc_phase_selector_temp.tof_dist_filtered[1]) > MPC_V_TOLERANCE_VAL)))
    { curr_phase = APPROACH; }
    
    else 
    { curr_phase = DOCKING; }
}

void mpc_input::init_matrix()
{
    PRINTF("\n\n MPC step #%ld started \n\n",curr_itr_step);
    if(mpc_rx.getOnlyIfNewData(mpc_rx_temp))
    {
        X_k = mpc_rx_temp.ekf_Xk;
        curr_itr_step = mpc_rx_temp.curr_time_step;
    }

}

void mpc_input::state_space_init()
{
    if(curr_time_step == 0)
    { 
        X_curr = (A * X_k) + (B * U);
        F = (X_curr - X_des);
        F_t = F.transpose();
        stsp_U_t = stsp_U.transpose();
    } 
    else
    {
        X_des = X_k;
        X_curr = (A * X_k) + (B * U);
        F = (X_curr - X_des);
        F_t = F.transpose();
        stsp_U_t = stsp_U.transpose();
    }    
}

void mpc_input::alignment_phase()
{
    PRINTF("Allignment Phase Activated");
    costfn_Q.r[0][0] = 0;
    costfn_Q.r[1][1] = 0;
    costfn_Q.r[2][2] = 10;
    costfn_Q.r[3][3] = 5;
    //costfn_Q.print();
    
    for(int i = 0 ; i < MPC_PREDICTION_HORIZON ; i++)        
        J += {(F_t * (costfn_Q * F)) + (stsp_U_t * (costfn_R * stsp_U))};
}

void mpc_input::approach_phase()
{
    PRINTF("Approach Phase Activated");
    costfn_Q.r[0][0] = 10;
    costfn_Q.r[1][1] = 5;
    costfn_Q.r[2][2] = 0;
    costfn_Q.r[3][3] = 0;
    //costfn_Q.print();

    for(int i = 0 ; i < MPC_PREDICTION_HORIZON ; i++)        
        J += {(F_t * (costfn_Q * F)) + (stsp_U_t * (costfn_R * stsp_U))};
}

void mpc_input::Docking_phase()
{
    PRINTF("Docking Phase Activated");
    costfn_Q.r[0][0] = 10;
    costfn_Q.r[1][1] = 5;
    costfn_Q.r[2][2] = 10;
    costfn_Q.r[3][3] = 5;
    //costfn_Q.print();

    for(int i = 0 ; i < MPC_PREDICTION_HORIZON ; i++)        
        J += {(F_t * (costfn_Q * F)) + (stsp_U_t * (costfn_R * stsp_U))};
}

void mpc_input::phase_switch_case()
{
    switch(curr_phase)
    {
    case ALLIGNMENT:
        alignment_phase();
        break;
    case APPROACH:
        approach_phase();
        break;
    case DOCKING:
        Docking_phase();
        break;
    default:
        PRINTF("Entered default case");
        break; 
    }       
}

void mpc_input::init()
{
    mpc_phase_selector();
    init_matrix();
    state_space_init();
}

void mpc_input::publish_mpc_result()
{
    

}

void mpc_input::run()
{
    for(int i = 0 ; i < N ; i++)
    { 
        phase_switch_case();
        publish_mpc_result();
    }
}