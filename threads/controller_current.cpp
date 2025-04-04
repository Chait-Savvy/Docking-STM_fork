#include "controller_current.h"

CommBuffer<tcmd_struct> tcmd_rx_temp;
Subscriber subs_tcmd(topic_tcmd_exe,tcmd_rx_temp);

#
static pid ctrl_mag[4];

double current_ctrl::compute_required_current_approach()
{
    // mpc_rx.getOnlyIfNewData(current_rx);
    mpc_rx.get(current_rx);
    float distance = *current_rx.ekf_Xk.r[0];
    float force_req = current_buff.buff_F.r[0][0];
    double denominator_sum = 0.0;
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            float pos1_y = positions.r[k][0];
            float pos1_z = positions.r[k][1];
            float pos2_y = positions.r[j][0];
            float pos2_z = positions.r[j][1];

            double y_diff = std::abs(pos1_y - pos2_y);
            double z_diff = std::abs(pos1_z - pos2_z);

            // Set to fixed values if not the same
            y_diff = (y_diff != 0) ? FIXED_Y_DIST : 0;
            z_diff = (z_diff != 0) ? FIXED_Z_DIST : 0;

            double rij = std::sqrt( (distance * distance) + (y_diff * y_diff) + (z_diff * z_diff));
            denominator_sum += 1.0 / pow(rij, 4);
        }
    }
    double K = (3 * MU_ZERO / (2 * M_PI)) * pow(N_COIL * A, 2);

    // Solve for current
    double I_required = sqrt(force_req / (K * denominator_sum));
    return I_required;  
}

double current_ctrl::sign(const double in)
{
  if (in < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

void current_ctrl::mag_pid_ctrl(magnet_idx idx)
{
    static int last_sign[4] = {1,1,1,1};
    float mag_i_val[4];
    double current_setpt = compute_required_current_approach();
    mag_status_temp.desired_current = current_setpt;
    // PRINTF(" %lf ", current_setpt);

    switch(idx)
    {
        case MAGNET_IDX_ALL:
            magnet::get_current(mag_i_val);
            for(uint8_t i = 0; i < 4; i++)
                {
                  mag_status_temp.i_dt_status[i] = mag_i_val[i];
                //   PRINTF(" %f ", mag_i_val[i]);
                  mag_i_val[i] = sqrtf(mag_i_val[i] * mag_i_val[i]);  
                  mag_i_val[i] = last_sign[i] * mag_i_val[i]; // Signed current
                  float error = current_setpt - mag_i_val[i];
                  float pwm = ctrl_mag[i].update(error, 50 / 1000.0);
                  magnet::actuate((magnet_idx)i, pwm);
                //   mag_status_temp.i_dt_status[i] = (int)(pwm);
                  last_sign[i] = sign(pwm); // Store sign
                }
                // PRINTF(" \n ");
            break;
        default :
             int idx_int = (int)(idx); 
             float mag_idx_curr = magnet::get_current(idx);
            //  PRINTF(" %f \n", mag_idx_curr);
             float error = current_setpt - mag_idx_curr;
             float pwm = ctrl_mag[idx_int].update(error, 50 / 1000.0);
             magnet::actuate(idx, pwm);
             mag_status_temp.i_dt_status[idx_int] = (int)(pwm);
             last_sign[idx_int] = sign(pwm); // Store sign   
             break;
    }
}

void current_ctrl::actuate_magnets()
{
    if(mag_on_flag)
    {
        allignment_phase L1 = current_rx.mpc_phase;   
        switch(L1)
        {
        case ALLIGNMENT :
            if(*current_rx.ekf_Xk.r[2] > 0.0)
            	{
                    mag_pid_ctrl(MAGNET_IDX_1);
                    mag_pid_ctrl(MAGNET_IDX_2);
                }
            else
                {
                    mag_pid_ctrl(MAGNET_IDX_0);
                    mag_pid_ctrl(MAGNET_IDX_3);
                }
            break;    
        case APPROACH :
            mag_pid_ctrl(MAGNET_IDX_ALL);
            break;
        case DOCKING :
            mag_pid_ctrl(MAGNET_IDX_ALL);    
            break;    
        }
    }

} 

void current_ctrl::publish_current_data()
{
    mpc_to_i_topic.publish(mag_status_temp);
    // PRINTF("\n %d %d %d %d \n", mag_status_temp.i_dt_status[0],mag_status_temp.i_dt_status[1],mag_status_temp.i_dt_status[2],mag_status_temp.i_dt_status[3]);
}

void current_ctrl::init()
{
    mpc_rx.getOnlyIfNewData(current_rx);
    mpc_ctrlinput_buff.getOnlyIfNewData(current_buff);
    magnet::init();
    for(uint8_t i = 0; i < 4; i++)
        {
          ctrl_mag[i].set_kp(PID_CURRENT_KP);
          ctrl_mag[i].set_ki(PID_CURRENT_KI);
          ctrl_mag[i].set_control_limits(PID_CURRENT_UMIN, PID_CURRENT_UMAX);
        }
}

void current_ctrl::run()
{
    TIME_LOOP( THREAD_ICTRL_START * MILLISECONDS , CURRENT_THREAD_PERIOD * MILLISECONDS)
    {
        actuate_magnets();
        publish_current_data();
    }
}
current_ctrl f_to_i("magnet thread",70,2000);