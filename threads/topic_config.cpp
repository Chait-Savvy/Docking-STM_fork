#include "topic_config.h"

Topic<itr_step_status> curr_itr_number_topic(-1,"Current iteration number running status");
Topic<tof_to_ekf> tof_to_ekf_topic(-1,"tof_and_position_data");
Topic<ekf_to_mpc> ekf_to_mpc_topic(-1,"tof_and_position_data");
Topic<mpc_buff> mpc_last_ctrlinput_buff_topic(-1,"mpc control input buffer");
Topic<status_and_desired_current> mpc_to_i_topic(-1,"Current Duty Cycle data");
Topic<ekf_buff> ekf_last_state_buff_topic(-1,"EKF last time step storage buffer");

CommBuffer<tof_to_ekf> ekf_rx;
Subscriber subs_ekf(tof_to_ekf_topic,ekf_rx);

CommBuffer<itr_step_status> ekf_itr_status;
Subscriber subs_ekf_itr(curr_itr_number_topic,ekf_itr_status);

CommBuffer<ekf_to_mpc> mpc_rx;
Subscriber subs_mpc(ekf_to_mpc_topic,mpc_rx);

CommBuffer<status_and_desired_current> magctrl_rx;
Subscriber subs_magctrl(mpc_to_i_topic,magctrl_rx);

CommBuffer<ekf_buff> ekf_last_state_buff;
Subscriber subs_ekf_buff(ekf_last_state_buff_topic,ekf_last_state_buff);

CommBuffer<mpc_buff> mpc_ctrlinput_buff;
Subscriber subs_mpc_buff(mpc_last_ctrlinput_buff_topic,mpc_ctrlinput_buff);

