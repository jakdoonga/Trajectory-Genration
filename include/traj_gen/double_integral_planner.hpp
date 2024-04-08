#ifndef DOUBLE_INTEGRAL_PLANNER_HPP
#define DOUBLE_INTEGRAL_PLANNER_HPP

#include <iostream>
#include <cmath>
#include <vector>


using std::cout;
using std::cin;
using std::endl;

class DoubleIntegralPlanner
{
    public:
        DoubleIntegralPlanner();
        DoubleIntegralPlanner(double a_max_, double v_max_);
        void TimeCalc();
        void goalTraj(double t);

        void setGoalPos(double &p_goal_ref);
        void setAcc_max(double &a_max_ref);
        void setVel_max(double &v_max_ref);


        double getPos();
        double getVel();
        double getAcc();

    private:
        double a_max;
        double v_max;

        double p_goal;

        double t_1;
        double t_2;
        double t_f;

        double a_curr;
        double v_curr;
        double p_curr;
};



#endif