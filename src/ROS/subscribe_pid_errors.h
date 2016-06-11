#ifndef SUBSCRIBE_PID_ERRORS_H
#define SUBSCRIBE_PID_ERRORS_H

#include <ros/ros.h>
#include "ark_controls/PidErrors.h"

#include "../shared_memory.h"
#include "../gui/threadgui.h"

class Subscribe_pid_errors
{
public:
    Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui);
    void pidErrorsCb(const ark_controls::PidErrorsConstPtr &msg);

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;
};
#endif // SUBSCRIBE_PID_ERRORS_H
