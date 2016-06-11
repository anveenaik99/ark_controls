#include "subscribe_pid_errors.h"

Subscribe_pid_errors::Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;    
}

void Subscribe_pid_errors::pidErrorsCb(const ark_controls::PidErrorsConstPtr &msg)
{
    if (this->shared_memory->getSharedControl() && this->shared_memory->getOverride())
    {

    }
}
