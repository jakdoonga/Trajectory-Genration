#include "double_integral_planner.hpp"

// Constructor without setting constraints
DoubleIntegralPlanner::DoubleIntegralPlanner()
{
    p_curr = 0;
    v_curr = 0;
    a_curr = 0;
}

// Constructor with constraints
DoubleIntegralPlanner::DoubleIntegralPlanner(double a_max_, double v_max_)
:a_max(a_max_), v_max(v_max_)
{
    p_curr = 0;
    v_curr = 0;
    a_curr = 0;
}

void DoubleIntegralPlanner::setGoalPos(double &p_goal_ref)
{
    p_goal = p_goal_ref;
    cout<<"Goal pos: "<<p_goal<<endl;
}

void DoubleIntegralPlanner::setAcc_max(double &a_max_ref)
{
    a_max = a_max_ref;
    cout << "acc max: "<<a_max<<endl;
}

void DoubleIntegralPlanner::setVel_max(double &v_max_ref)
{
    v_max = v_max_ref;
    cout << "vel max: "<<v_max<<endl;
}

void DoubleIntegralPlanner::TimeCalc() {
    
    t_1 = v_max/a_max;
    t_2 = (p_goal/v_max);
    t_f = t_1 + t_2;

    if (t_1 >= t_2) {
        t_1 = sqrt(p_goal/a_max);
        t_2 = t_1;
        t_f = 2*t_1;
        v_max = a_max*t_1;
        cout << "Double Integral:: Bang-Bang" << endl;
    }
    else {
        cout << "Double Integral:: Bang-off-Bang" << endl;
    }
}

void DoubleIntegralPlanner::goalTraj(double t) {
    if ( t < 0)
    {
        p_curr = 0;
        v_curr = 0;
        a_curr = 0;
        // cout<<"Negative time"<<endl;
    }
    else if (t >= 0 && t <= t_1) {
        a_curr = a_max;
        v_curr = a_curr*t;
        p_curr = 0.5*v_curr*t;
    }
    else if (t <= t_2) {
        a_curr = 0;
        v_curr = a_max*t_1;
        p_curr = 0.5*v_max*t_1 + v_curr*(t-t_1);
    }
    else if (t <= t_f) {
        a_curr = -a_max;
        v_curr = a_max*t_1 + a_curr*(t-t_2);
        p_curr = 0.5*v_max*t_1 + v_max*(t_2-t_1) + 0.5*(v_max+v_curr)*(t-t_2);
    }
    else if (t >= t_f){
        a_curr = 0;
        v_curr = 0;
        p_curr = 0.5*v_max*t_1 + v_max*(t_2-t_1) + 0.5*v_max*(t_f-t_2);
    }
}

double DoubleIntegralPlanner::getPos()
{
    return p_curr;
}

double DoubleIntegralPlanner::getVel()
{
    return v_curr;
}

double DoubleIntegralPlanner::getAcc()
{
    return a_curr;
}