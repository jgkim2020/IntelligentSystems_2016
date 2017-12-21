#include <project4/purePursuit.h>

purePursuit::purePursuit(){

}

control purePursuit::get_control(point x_robot, point x_goal){

    /* TO DO
     *
     * implement purepursuit algorithm
     * specification for pioneer robot: maximum velocity - 1.2m/s, maximum angular velocity - 300dps (~5.23rad/s)
     *
    */
    control ctrl;
    double x = cos(x_robot.th-(M_PI/2))*(x_goal.x-x_robot.x)+sin(x_robot.th-(M_PI/2))*(x_goal.y-x_robot.y);
    double y = -sin(x_robot.th-(M_PI/2))*(x_goal.x-x_robot.x)+cos(x_robot.th-(M_PI/2))*(x_goal.y-x_robot.y);
    double angle = atan2(x,y);
    double threshold = 23*M_PI/180;
    double max_w = 0.5;
    if(angle < threshold && -threshold < angle)
    {
        ctrl.v = 0.19;
        ctrl.w = -(ctrl.v*2*x)/(x*x+y*y);
    }
    else
    {
        ctrl.v = 0;
        ctrl.w = -angle;
        /*if(angle > 1) ctrl.w = max_w;
        else if(angle < -1) ctrl.w = -max_w;
        else ctrl.w = max_w*angle;*/
    }

    return ctrl;
}
