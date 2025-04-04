/* Config file to define sampling time, size of matrix and other parameters for the 
Tof_measuremnt block, Tof_filter block, MPC_controller block   */

#ifndef _CONFIG_MPC_H_
#define _CONFIG_MPC_H_

/* 
    TOF measurement and EKF filter thread variables  
                                                    
     schematics of indices
     _____________       ______ ______
    |             |     |      -      |
    |  1       0  |     |  3       2  | 
    | (O)     (O) |     | (O)     (O) |
    |             |     |             |
    |      1      |     |      3      |
    |      o      |     |      o      |
    |             |     |             |
    |             |     |             |
    | 0 o     o 2 |     | 2 o     o 0 |
    |             |     |             |
    |             |     |             |
    |      o      |     |      o      |
    |      3      |     |      1      |
    |             |     |             |
    |             |     |             |
    |  2       3  |     |  0       1  |
    | (O)     (O) |     | (O)     (O) |
    |______-______|     |_____________|

(O) - electromagnet
 o  - time of flight sensor                     */

/* I/O configuration */ 



 /* Math constants */

#define MU_ZERO 4 * M_PI * 1e-7                // H/m

/* Coil paramaters */

#define N_COIL 600                                  // Number of turns per coil     
#define RC 0.016                               // Active radius of coils
#define A  M_PI * RC * RC                      // Effective cross-sectional area 

/*  Satellite physical parameters */ 

#define TAMARIW_SATELLITE_MASS 4               // Satellite mass
#define TAMARIW_SATELLITE_MOI 0.03             // Satellite MoI
#define TAMARIW_NEAR_TOF_DISTANCE 10          // Distance between near side TOF sensor in MILLIS

/* TOF and position determination block parameters */


#define TOF_R2D 57.2957795131
#define TOF_D2R 0.01745329251
#define TOF_FIRST_ROLL_VELOCITY true
#define TOF_FIRST_PITCH_VELOCITY true
#define TOF_LAST_PITCH 0.00
#define TOF_LAST_ROLL 0.00


/*EKF Prediction Model VARIABLES  */           

#define EKF_TOF_SENSORS_REQ 4                // Number of required sensors for TOF, 0<N<=  
#define EKF_TOF_INIT_SENSOR_SEQUENCE TOF_IDX_0,TOF_IDX_1,TOF_IDX_3,TOF_IDX_2  // sensor on(1)/off(0) sequence as bits in string : #1,#2,#3,#4   
#define EKF_TOF_SAMP_TIME_DT 0.1             // Sampling time is in Seconds  
#define EKF_STATE_VECTOR_SIZE 4
#define EKF_STATE_TRANSITION_MATRIX_SIZE 4,4  // defines M for the square matrix -> A[M][M] 
#define EKF_CONTROL_INPUT_MATRIX 4           // defines N for the matrix -> B[M][N]
#define EKF_CONTROL_INPUT_VECTOR 4
#define EKF_STATE_ESTIMATE_NOISE_COVARIANCE_VECTOR 4        //Q(k) process noise with covariance Q.

/*EKF Measurement Model VARIABLES */

#define EKF_MEASUREMENT_MATRIX_SIZE 4,4        //  defines N for the square matrix -> W[N][N]
#define EKF_MEASUREMENT_MATRIX_VECTOR 4      //  v(k) ∼ N(0,R): measurement noise with covariance R.

/* MPC iteration variables */

#define MPC_PREDICTION_HORIZON 20            //Defines prediction horizon N
#define MPC_SAMPLING_TIME_MILLIS 50         // dT
#define MPC_STATE_SPACE_SIZE MPC_PREDICTION_HORIZON
#define MPC_CONTROL_INPUT_VECTOR_SIZE (2*MPC_PREDICTION_HORIZON)
#define MPC_CONTROL_INPUT_SIZE 2
#define MPC_X_DES 0
#define MPC_V_DES 0 
#define MPC_THETA_DES 0
#define MPC_OMEGA_DES 0  
#define MPC_X_TOLERANCE_VAL 25                // Position tolerance [mm] (for docking)
#define MPC_V_TOLERANCE_VAL 0.1               // Velocity tolerance [mm/s] (for docking)
#define MPC_THETA_TOLERANCE_VAL 0.1           // Theta tolerance [rad||__deg__] (for docking)

/* Force to Current conversion block */

#define FIXED_Z_DIST 0.252
#define FIXED_Y_DIST 0.041
#define DISTANCE_CONVERSION_FACTOR 1       //If dist in mm -> 1 , cm -> 10 , mtr -> 1000
#define MAX_LOOKUP_TABLE_DIST_RANGE 300    
#define IDX0_X (-1 * FIXED_Y_DIST)/2
#define IDX0_Y (-1 * FIXED_Z_DIST)/2
#define IDX1_X (1 * FIXED_Y_DIST)/2
#define IDX1_Y (-1 * FIXED_Z_DIST)/2
#define IDX2_X (-1 * FIXED_Y_DIST)/2
#define IDX2_Y (1 * FIXED_Z_DIST)/2
#define IDX3_X (1 * FIXED_Y_DIST)/2
#define IDX3_Y (1 * FIXED_Z_DIST)/2

#define PID_CURRENT_UMAX 60
#define PID_CURRENT_UMIN 0
#define PID_CURRENT_KP 0.065
#define PID_CURRENT_KI 0.3

/* Thread timming */

#define TIME_PERIOD_PER_THREAD 10
#define THREAD_TOF_START 50
#define MASTER_THREAD_START_TIME (THREAD_TOF_START+TIME_PERIOD_PER_THREAD)
#define THREAD_TELEMETRY_START (THREAD_TOF_START+TIME_PERIOD_PER_THREAD)
#define THREAD_TCMD_START (THREAD_TELEMETRY_START+TIME_PERIOD_PER_THREAD)

#define THREAD_EKF_START (THREAD_TOF_START+TIME_PERIOD_PER_THREAD)
#define THREAD_MPC_START (THREAD_EKF_START+TIME_PERIOD_PER_THREAD)
#define THREAD_ICTRL_START (THREAD_MPC_START+TIME_PERIOD_PER_THREAD)
#define CURRENT_THREAD_PERIOD 10

#endif //config_mpc.h