#include "controller_current.h"



data_desired_current current_ctrl::convert_f_to_i(float d , float f , allignment_phase L1)
{
    struct data_desired_current C1;
    if(d < MAX_LOOKUP_TABLE_DIST_RANGE)
            int idx = {(int)(d * DISTANCE_CONVERSION_FACTOR *2)};
    switch(L1)
    {
        case ALLIGNMENT :  
            int i_interpolated = ((100 * f)/Allign_Lookup.r[idx]);
            C1.i[] = {0,i_interpolated,i_interpolated,0};
            magnet::actuate(MAGNET_IDX_1,i_interpolated);
            magnet::actuate(MAGNET_IDX_2,i_interpolated);            
            break;
        case APPROACH :
            int i_interpolated = ((100 * f)/Approach_Lookup.r[idx]);
            C1.i[] = {i_interpolated,i_interpolated,i_interpolated,i_interpolated};
            magnet::actuate(MAGNET_IDX_ALL,i_interpolated)
            break;
        case DOCKING :
            int i_interpolated = ((100 * f)/Approach_Lookup.r[idx]);
            C1.i[] = {i_interpolated,i_interpolated,i_interpolated,i_interpolated};
            magnet::actuate(MAGNET_IDX_ALL,(i_interpolated*-1))
            break; 
    }    
    	
    return C1;
}

void current_ctrl::publish_current_data()
{
    // CommBuffer<data_desired_current> current1;


}

void current_ctrl::init()
{
    magnet::init();
}

void current_ctrl::run()
{
    current1 = convert_f_to_i(dist,mpc_current_req,L1);
    publish_current_data();
}