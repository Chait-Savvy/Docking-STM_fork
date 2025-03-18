#include "filter_ekf.h"

CommBuffer<tof_to_ekf> ekf_rx;
Subscriber subs_ekf(tof_to_ekf_topic,ekf_rx);

CommBuffer<itr_step_status> ekf_itr_status;
Subscriber ekf_itr(curr_itr_number_topic,ekf_itr_status);

itr_step_status ekf_itr_status_temp;
tof_to_ekf ekf_rx_temp;
ekf_to_mpc ekf_tx_temp;

void ekf_block::init_matrix()
{
    PRINTF("\n\n EKF step #%ld started \n\n",itr_step);
    if(ekf_rx.getOnlyIfNewData(ekf_rx_temp))
    {
        for(int i = 0 ; i < 4 ; i++)
            dist[i] = ekf_rx_temp.tof_dist_filtered[i];
        itr_step = ekf_itr_status_temp.curr_iteration_step_ekf;
        PM_Xk = ekf_rx_temp.Xk;
        MM_Zk = ekf_rx_temp.Zk_filtered;
    }
}

void ekf_block::pred_model()
{
    if(itr_step == 0)
    {
      PM_Xk = MM_Zk;
    } 
    // PM_Xk.print();
    PM_Xk = {( PM_A * PM_Xk) + (PM_B * PM_Uk) + (PM_V)};
    PM_P = {(PM_A * (PM_P * (PM_A.transpose()))) + PM_Q};
    // PM_Xk.print();
    // PM_P.print();
    // PRINTF("\n");
    // PRINTF("prediction step #%ld complete \n",time_step);
}

void ekf_block::update_step()
{
    MM_Yk = ( MM_Zk - ((MM_Hk * PM_Xk) + MM_Wk));
    MM_Hk_T = MM_Hk.transpose();
    MM_Sk_I = MM_Sk.invert();
    MM_Sk = (( MM_Hk * PM_P * MM_Hk_T ) + MM_Rk);
    MM_Kk = (( PM_P * MM_Hk_T ) * (MM_Sk.invert()));
    // PM_I.eye(1);
    // PM_P.print();
    PM_Xk = (PM_Xk + ( MM_Kk * MM_Yk));
    PM_P = ((PM_I - MM_Kk * MM_Hk) * PM_P );
    itr_step += 1;
//     MM_Kk.print();
//     PM_Xk.print();
//     PM_P.print();
//     PRINTF("\n");
//     PRINTF("Correction step #%ld complete \n",itr_step);
}

void ekf_block::publish_results()
{
    ekf_itr_status_temp.curr_iteration_step_ekf = itr_step;
    ekf_tx_temp.ekf_Xk = PM_Xk;
    ekf_to_mpc_topic.publish(ekf_tx_temp);
    curr_itr_number_topic.publish(ekf_itr_status_temp); 
}

void ekf_block::init()
{
    //print_all_mtrx();
}

void ekf_block::run()
{
    TIME_LOOP(100 * MILLISECONDS, 1000 * MILLISECONDS)
    {
      init_matrix();  
      pred_model();
      update_step();
      publish_results();
      PRINTF("EKF step #%ld complete \n\n",itr_step);
      PRINTF("\n\n _______________________________________________________");
      next_time_step();
    }   
}


