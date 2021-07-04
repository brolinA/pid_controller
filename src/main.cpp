#include <iostream>
#include <../include/pid_smd.h>

using namespace std;

int main(int argc, char **argv)
{   
    if(argc == 4 )
    {
        double kp_, ki_, kd_;
        kp_ = atof(argv[1]);
        ki_ = atof(argv[2]);
        kd_ = atof(argv[3]);
        pid_smd::pidSMDSystem pid_(kp_, ki_, kd_);
    }
    else
        pid_smd::pidSMDSystem pid_;

    return 0;
}
