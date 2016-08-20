#ifndef SUBSCRIBE_PID_ERRORS_H
#define SUBSCRIBE_PID_ERRORS_H

#include <ros/ros.h>
#include "ark_msgs/PidErrors.h"
#include <math.h>

#include "../shared_memory.h"
#include "../gui/threadgui.h"

class Subscribe_pid_errors
{
public:
    Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui);
    void pidErrorsCb(const ark_msgs::PidErrorsConstPtr &msg);

    float kpx;
    float kix;
    float kdx;
    float kpy;
    float kiy;
    float kdy;
    float accelmax_x;
    float accelmax_y;
    float deaccelmax_x;
    float deaccelmax_y;
    float vmax_x;
    float vmax_y;
    float inr;
    float x3;
    float y3;

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;

    float prev_time;
    float sumerrorvx;
    float sumerrorvy;
    float preverrorvx;
    float preverrorvy;
    float targetv_x;
    float targetv_y;
    float cout_prev_time;
};
#endif // SUBSCRIBE_PID_ERRORS_H
