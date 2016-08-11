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

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;

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


    float prev_time;
    float x3;
    float y3;
    float sumerrorvx;
    float sumerrorvy;
    float preverrorvx;
    float preverrorvy;
};
#endif // SUBSCRIBE_PID_ERRORS_H
