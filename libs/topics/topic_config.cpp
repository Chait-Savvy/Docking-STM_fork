#include "topic_config.h"
#include "rodos.h"

Topic<itr_step_status> curr_itr_number_topic(-1,"Current iteration number running status");
Topic<tof_to_ekf> tof_to_ekf_topic(-1,"tof_and_position_data");
Topic<ekf_to_mpc> ekf_to_mpc_topic(-1,"tof_and_position_data");
Topic<mpc_to_current_controller> mpc_to_i_topic(-1,"tof_and_position_data");
Topic<status_and_desired_current> status_and_desired_current_topic(-1,"Current Duty Cycle data");
