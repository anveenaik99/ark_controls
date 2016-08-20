#include "subscribe_pid_errors.h"

Subscribe_pid_errors::Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;

    kpx = 0.0; // set values # 600
    kix = 0.0; // set values
    kdx = 0.0; // set values # 100
    kpy = 0.0; // set values # 600
    kiy = 0.0; // set values 
    kdy = 0.0; // set values # 100
    accelmax_x = 0.45;
    accelmax_y = 0.45;
    deaccelmax_x = 0.1;
    deaccelmax_y = 0.1;
    vmax_x = 0.5;
    vmax_y = 0.5;
    inr = 0.02;


    prev_time = 0.0;
    cout_prev_time = 0.0;
    x3 = (vmax_x * vmax_x)/(2*deaccelmax_x) + inr;
    y3 = (vmax_y * vmax_y)/(2*deaccelmax_y) + inr;
    sumerrorvx = 0.0;
    sumerrorvy = 0.0;
    preverrorvx = 0.0;
    preverrorvy = 0.0;
}

void Subscribe_pid_errors::pidErrorsCb(const ark_msgs::PidErrorsConstPtr &msg)
{
    float current_time = ros::Time::now().toNSec() / 1000000000.0; // Current time
    if (this->shared_memory->getSharedControl() && this->shared_memory->getOverride())
    {
        float errorx = msg->dx;
        float errory = msg->dy;
        float errorpsi = msg->dpsi;
        float errorz = msg->dz;
        float velx = msg->vx;
        float vely = msg->vy;

        
        if (prev_time == 0)
        {
            prev_time = current_time;
            cout_prev_time = current_time;
            return;
        }

        float del_time = current_time - prev_time;

        int x_direction = (errorx > 0) ? -1 : 1;
        int y_direction = (errory > 0) ? -1 : 1;

        errorx = fabs(errorx);
        errory = fabs(errory);

        if (errorx < inr)
        {
            targetv_x = 0;
            sumerrorvx = 0;
        }
        else
        {
            if (errorx <= x3) 
            {
                targetv_x = sqrt(fabs(vmax_x * vmax_x - 2 * deaccelmax_x * (x3 - errorx)));
                //std::cout<<x3<<" : "<<x_direction<<std::endl;
            }
            else targetv_x = vmax_x;
            targetv_x = targetv_x * x_direction;
            if (fabs(targetv_x - velx) > accelmax_x * del_time) targetv_x = accelmax_x * x_direction * del_time + velx;            
        }
        if (errory < inr)
        {
            targetv_y = 0;
            sumerrorvy = 0;
        }
        else
        {
            if (errory <= y3)
            {
                targetv_y = sqrt(fabs(vmax_y * vmax_y - 2 * deaccelmax_y * (y3 - errory)));
                //std::cout<<x3<<" : "<<x_direction<<std::endl;
            }
            else targetv_y = vmax_y;
            targetv_y = targetv_y * y_direction;
            if (fabs(targetv_y - vely) > accelmax_y * del_time) targetv_y = accelmax_y * y_direction * del_time + vely;
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

        if(current_time - cout_prev_time > 0.5)
        {
            std::cout<<targetv_x<<","<<velx<<","<<errorx<<","<<errorvx<<","<<PIDvx<<std::endl;
            cout_prev_time = current_time;
        }
        
        //else this->t_gui->gui->channel12->setYValue(1500);
        // end X PID

        // Y PID
        float PIDvy = (kpy*errorvy + kiy*sumerrorvy + (kdy*(preverrorvy - errorvy))) + 1500;
        if(PIDvy>1700) PIDvy = 1700;
        if(PIDvy<1300) PIDvy = 1300;
        this->t_gui->gui->channel12->setXValue(PIDvy);
        sumerrorvy = sumerrorvy + errorvy;
        preverrorvy = errorvy;        
        //else this->t_gui->gui->channel12->setXValue(1500);
        // end Y PID         
    }
    // Clean Up
    prev_time = current_time;   
}
