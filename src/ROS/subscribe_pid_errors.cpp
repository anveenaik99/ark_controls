#include "subscribe_pid_errors.h"

Subscribe_pid_errors::Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;

    kpx = 250.0; // set values
    kix = 0.0; // set values
    kdx = 0.0; // set values
    kpy = 2.0; // set values
    kiy = 0.0; // set values
    kdy = 0.0; // set values
    accelmax_x = 0.7;
    accelmax_y = 0.5;
    deaccelmax_x = 0.7;
    deaccelmax_y = 0.5;
    vmax_x = 0.7;
    vmax_y = 0.7;
    inr = 0.02;


    prev_time = 0.0;
    x3 = (vmax_x * vmax_x)/(2*deaccelmax_x);
    y3 = (vmax_y * vmax_y)/(2*deaccelmax_y);
    sumerrorvx = 0.0;
    sumerrorvy = 0.0;
    preverrorvx = 0.0;
    preverrorvy = 0.0;
}

void Subscribe_pid_errors::pidErrorsCb(const ark_msgs::PidErrorsConstPtr &msg)
{
    if (this->shared_memory->getSharedControl() && this->shared_memory->getOverride())
    {
        float errorx = msg->dx;
        float errory = msg->dy;
        float errorpsi = msg->dpsi;
        float errorz = msg->dz;

        float velx = msg->vx;
        float vely = msg->vy;

        float current_time = ros::Time::now().toNSec() / 1000000; // Current time
        if (prev_time == 0)
        {
            prev_time = current_time;
            return;
        }

        int del_time = current_time - prev_time;

        int x_direction = (errorx > 0) ? -1 : 1;
        int y_direction = (errory > 0) ? -1 : 1;

        errorx = fabs(errorx);
        errory = fabs(errory);

        float targetv_x;
        float targetv_y;

        if (errorx < inr) targetv_x = 0;
        else
        {
            if (errorx <= x3) targetv_x = sqrt(vmax_x * vmax_x + 2 * deaccelmax_x * (x3 - errorx));
            else targetv_x = vmax_x;
            if (targetv_x - velx > accelmax_x * del_time) targetv_x = accelmax_x * del_time + velx;
            targetv_x = targetv_x * x_direction;
        }
        if (errory < inr) targetv_y = 0;
        else
        {
            if (errory <= y3) targetv_y = sqrt(vmax_y * vmax_y + 2 * deaccelmax_y * (y3 - errory));
            else targetv_y = vmax_y;
            if (targetv_y - vely > accelmax_y * del_time) targetv_y = accelmax_y * del_time + vely;
            targetv_y = targetv_y * y_direction;
        }

        float errorvx = targetv_x - velx;
        float errorvy = targetv_y - vely;

        // Alt PID
        if (fabs(errorz) > 0.2)
        {
            if (errorz > 0) this->t_gui->gui->channel34->setYValue(1340);
            else if (errorz < 0) this->t_gui->gui->channel34->setYValue(1660);
        }            
        else this->t_gui->gui->channel34->setYValue(1500);
        // End Alt Pid   

        // X PID
        float PIDvx = 1500 - (kpx*errorvx + kix*sumerrorvx + (kdx*(preverrorvx - errorvx)));
        if(PIDvx>1700) PIDvx = 1700;
        if(PIDvx<1300) PIDvx = 1300;
        this->t_gui->gui->channel12->setYValue(PIDvx);
        sumerrorvx = sumerrorvx + errorvx;
        preverrorvx = errorvx;
        std::cout<<PIDvx<<std::endl;
        
        //else this->t_gui->gui->channel12->setYValue(1500);
        // end X PID

        // Y PID
        if(errorvy>0.02)
        {
            float PIDvy = kpy*errorvy + kiy*sumerrorvy + (kdy*(preverrorvy - errorvy)/del_time) + 1500;
            if(PIDvy>1800) PIDvy = 1800;
            if(PIDvy<1300) PIDvy = 1300;
            //this->t_gui->gui->channel12->setYValue(PIDvy);
            sumerrorvy = sumerrorvy + errorvy;
            preverrorvy = errorvy;
        }
        //else this->t_gui->gui->channel12->setYValue(1500);
        // end Y PID


        // Clean Up
        prev_time = current_time;    
    }
}
