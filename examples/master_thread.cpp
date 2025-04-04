#include "rodos.h"
#include "thread.h"
#include "tof_range.h"
#include "telemetry.h"
#include "telecommand.h"


class master_thread : public StaticThread<4000>
{
public :

    // master_thread(const char* t_name , const int prio , const int st_size) : Thread(t_name,prio,st_size){} 
    master_thread(const char* t_name) : StaticThread(t_name){} 

    void init()
    {
        HAL_UART WIFI_1(UART_IDX3, GPIO_027, GPIO_026, GPIO_INVALID, GPIO_INVALID);
    }
    void run()
    {
        TIME_LOOP( MASTER_THREAD_START_TIME * MILLISECONDS , TIME_PERIOD_PER_THREAD * MILLISECONDS )
        {
            int64_t time = NOW();
            // tof_range_thread tamariw_tof_range_thread("tof_range_thread",100);
            telemetry_thread tamariw_telemetry_thread("telemetry_thread", THREAD_PRIO_TELEMETRY, 4000);
            telecommand_thread tamariw_telecommand_thread("telecommand_thread", THREAD_PRIO_TELECOMMAND);
            int64_t thread_time = (NOW()-time);
            PRINTF("Master thread timming is %lld",thread_time);
        }
    }
};

master_thread master_thread_run("Master Thread");