#ifndef _FILTER_EKF_H_
#define _FILTER_EKF_H_


#include "rodos.h"
#include "matlib.h"
#include "config_mpc.h"
#include "topic_config.h"
#include "topic_config.h"


   /* 
   tof_idx idx_list[EKF_TOF_SENSORS_REQ] = {EKF_TOF_INIT_SENSOR_SEQUENCE};
   tof_status status;
   */ 
  
class ekf_block : public Thread
{

  /* System Dynamics (Prediction Model) Variables */

  RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> PM_Xk{(float []){},"State vector X(k)"};
  RODOS::Matrix_F<EKF_STATE_TRANSITION_MATRIX_SIZE> PM_A{(float []){1,EKF_TOF_SAMP_TIME_DT,0,0,0,1,0,0,0,0,1,EKF_TOF_SAMP_TIME_DT,0,0,0,1},"State transition matrix A"};
  RODOS::Matrix_F<EKF_STATE_VECTOR_SIZE,EKF_CONTROL_INPUT_VECTOR> PM_B{(float []){0,0,0,0,(EKF_TOF_SAMP_TIME_DT/TAMARIW_SATELLITE_MASS),0,0,0,0,0,0,0,0,(EKF_TOF_SAMP_TIME_DT/TAMARIW_SATELLITE_MOI),0,0},"Control Input matrix B"};
  RODOS::Vector_F<EKF_CONTROL_INPUT_VECTOR> PM_Uk{(float []){},"Control Input vector U(k)"};
  RODOS::Vector_F<EKF_STATE_VECTOR_SIZE> PM_V{(float []){},"Process Noise Matrix_F V(k)"};
  RODOS::Matrix_F<EKF_STATE_TRANSITION_MATRIX_SIZE> PM_P{(float []){1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},"Predicted State Covariance Matrix_F P(k)(Default to EYE(0,1))"};
  RODOS::Matrix_F<EKF_STATE_TRANSITION_MATRIX_SIZE> PM_Q{(float []){0.01,0,0,0,0,0.1,0,0,0,0,0.0000076,0,0,0,0,0.0000003},"State Model Noise Covariance Matrix_F Q(k)"};
  const RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> PM_I{(float []){1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},"EYE<N,N>"};

  /* Measurement Model VARIABLES */

  RODOS::Vector_F<EKF_MEASUREMENT_MATRIX_VECTOR> MM_Yk{(float []){},"Measurement Residual Y(k)"};
  RODOS::Vector_F<EKF_MEASUREMENT_MATRIX_VECTOR> MM_Zk{(float []){0.7,0,0.08,0},"Sensor Measurement Z(k)"};
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Hk{(float []){1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},"Measurement Matrix_F H(k)"};
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Hk_T{(float []){},"Measurement Matrix_F Transpose H(k)^T"};             
  RODOS::Vector_F<EKF_MEASUREMENT_MATRIX_VECTOR> MM_Wk{(float []){},"Sensor Noise Assumption W(k)"};
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Sk{(float []){},"Measurement residual Covariance Matrix_F S(k)"};
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Sk_I{(float []){},"Measurement residual Covariance Matrix_F Inverse S(k)^(-1)"};           
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Rk{(float []){0.0025,0,0,0,0,0.04,0,0,0,0,0.00122,0,0,0,0,0.000076},"Sensor Measurement Noise Covariance Matrix_F R(k)"};
  RODOS::Matrix_F<EKF_MEASUREMENT_MATRIX_SIZE> MM_Kk{(float []){},"Kalman Gain K(k)"};
  
 private :  

   static int dist[4];
   static long int itr_step = 0;

 public :
     ekf_block(const char* thread_name , const int priority , const int stackSize) : Thread(thread_name,priority,stackSize) {}

     void init_matrix();

     void pred_model();

     void update_step();

     void publish_results();

    void init();

    void run();

};

#endif  //ekf.h