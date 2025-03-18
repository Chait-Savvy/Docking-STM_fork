#include "rodos.h"
#include "thruster.h"
#include "thread.h"
#include "hal_uart.h"
#include <string.h>

#define TIME_PEROIOD_1_MS 100
#define TIME_PEROIOD_2_MS 500

#define UART_BUF_SIZE 40

uint16_t timekeeper_1 = 0;  
uint16_t timekeeper_2 = 0;

thruster t1;
//HAL_UART uart3(UART_IDX2);
// HAL_UART uart3(UART_IDX3);

HAL_GPIO led2(GPIO_061);
HAL_GPIO led3(GPIO_062);


// const char msg1[] = "UART3 traerwrwerensmitting \n";

class thruster_testing : public Thread
{   
    private : 

      PWM_IDX PWM1(PWM_IDX05);          // pwm1 -> PA1 Tim2 CH2
      GPIO_PIN EN(GPIO_017);            // en -> PB1   
      GPIO_PIN ENB(GPIO_018);           // enb -> PB2
      GPIO_PIN LED_BLUE(GPIO_063);      // led_blue -> PD15
      GPIO_PIN PSH_BTN(GPIO_000);       // blue push button -> PA0}

      int firing_period = 100;
      
    public :       

       thruster_testing(const char* thread_name) : Thread(thread_name)
       {
         init();
         run();
       }
      
       char msg  = 'A';

       void init()
       { 
          led2.init(true,1,0);
          led3.init(true,1,0);
          led2.setPins(1);
          led3.setPins(1);
          // uart3.init(115200);
          //uart3.config(UART_PARAMETER_BAUDRATE,115200);
          // uart3.write(&msg,siye);
       }
       
       void run()
       {

        init();
        // const char test[]="Testing Pi Serial \n\r";
		    //write(test,sizeof(test)-1);
        // uart3.write(&msg1[0], sizeof(msg1));
        PRINTF("CHECK");
        // TIME_LOOP( 0 * SECONDS , 1000 * MILLISECONDS)
        // {
        //   // PRINTF("Hello \n");
          
        //     // if(t1.init_flag()==true)
        //     //  {
        //     //   t1.fire_thruster(1000,100,5);
        //     //  }
        //  } 

       }
    
         
};

thruster_testing test1("thruster_thread");