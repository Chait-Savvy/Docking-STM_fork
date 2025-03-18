#include "rodos.h"
#include "matlib.h"
#include "controller_mpc.h"
#include "thread.h"
// #include "config_mpc.h"

// mpc_input *ctrl1;

class mpc_thread : public Thread,mpc_input
{ 
    private :

        // Matrix_<4,4> C{{},(char[]){"C"}};
        // Matrix<4,4> F{(double []){1,1,1,1,1,1,1,1,1},"F"};
        // Matrix<3,3> G{(double []){8,8,8,8,8,8,8,8,8},"G"};
        // Vector_<6> U{(const char[]){"U"}};
        Matrix_<4,4> A{(double []){1,0,0,0,1,0,0,0,1,7,8,5,4,2,4,2},"A"};
        Matrix_<2,2> B{(double []){1,0,3,3},"B"};
        Vector_<6> U{(double []){1,2,3,4,5,6}};
        Vector_<6> V{(const char[]){"U"}};
        Vector_<2> M{(double []){1500,15264}};
        allignment_phase phase = ALLIGNMENT;

        mpc_input ctrl1(5.00,100.00,A,B,U,V,M,phase);

    public :
    mpc_thread(const char* th_name , double n , double dT , Matrix<4,4> K , Matrix<2,2> L , Vector<6> x_des , 
        Vector<6> x_curr , Vector<2> mass_Param , allignment_phase p1) :
        Thread(th_name), ctrl1
    {
        
        void init();
        PRINTF("mpc thread initialised");
        void run();
    }

    void init()
    { }

    void run()  
    {
        PRINTF("Entered run()");    
        TIME_LOOP(0 * MILLISECONDS, 1000 * MILLISECONDS)
        {
            ctrl1.phase_switch_case();
        }        
    }
};

mpc_thread thread1("test mpc 1");