#include "subscribe_pid_errors.h"

Subscribe_pid_errors::Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;    
}

void Subscribe_pid_errors::pidErrorsCb(const ark_msgs::PidErrorsConstPtr &msg)
{
    if (this->shared_memory->getSharedControl() && this->shared_memory->getOverride())
    {
        float errorx = msg->dx;
        float errory = msg->dy;
        float errorpsi = msg->dpsi;
        float errorz = msg->dz;
        
        // Alt PID
        if (abs(errorz) > 0.2)
        {
            if (errorz > 0) this->t_gui->gui->channel34->setYValue(1340);
            else if (errorz < 0) this->t_gui->gui->channel34->setYValue(1660);
        }            
        else this->t_gui->gui->channel34->setYValue(1500);
        // End Alt Pid
    }
}
