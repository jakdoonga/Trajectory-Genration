#ifndef TRAJ_GEN_HPP
#define TRAJ_GEN_HPP

#include "traj_gen_definition.hpp"
#include "double_integral_planner.hpp"
#include "yaml_read.hpp"

#include <ros/ros.h>

#include <traj_gen/mode.h>

#include <msg_pkg/target.h>
#include <msg_pkg/target_dxl.h>

#include <msg_pkg/actual.h>
#include <msg_pkg/actual_dxl.h>

#include <geometry_msgs/Twist.h>

using std::string;

using traj_gen::mode;
using geometry_msgs::Twist;

using msg_pkg::target;
using msg_pkg::target_dxl;
using msg_pkg::actual;

typedef struct TRJ_DATA_{
    double a_curr[6];
    double v_curr[6];
    double p_curr[6];
} TRJ_DATA;


class Traj_Generator{

    public:

        Traj_Generator();

        // Get yaml directory
        void get_yaml_dir();

        // Trajectory constraint setup
        void constraint_setup();

        /**
         * Callback functions
        */

        // 1. Mode value callback function
        void callbackModeVal(const mode::ConstPtr& mode_ref);

        // 2. Motors info callback function
        void callbackActual(const actual::ConstPtr& actual_ref);

        // 3. Twist call back function
        void callbackTwist(const Twist::ConstPtr& twist_ref);

        /**
         * Movement member functions according
         * to mode value
        */


        // 1. Set Steering goal position 
        // before LIFT and WHEEL move.
        // mode value: 1
        void set_DXL_BEFORE_LIFT();

        // 2. Set LIFT goal position.
        // mode value: 2
        void set_LIFT();

        // 3. Set Steering position 
        // before PAN and WHEEL move.
        // mode value: 3
        void set_DXL_BEFORE_PAN();

        // 4. Set PAN goal position.
        // mode value: 4
        void set_PAN();

        // move motors
        void move_motors();

        // Move Lift motors
        void move_LIFT_motors();

        // Move Pan motors
        void move_PAN_motors();

        /**
         * DIP setup and get trajectory for idx
        */

        // idx: 0 ~ 2: PAN, idx: 3 ~ 5: LIFT
        // Firstly, set goal position for each idx
        void setGoalPos(double goal_pos, int idx);

        // Lastly, get position, velocity for each idx
        void getTraj(TRJ_DATA &traj_ref, int idx, double time);

        ~Traj_Generator();

    private:

        ros::NodeHandle nh_;


        // Declare dip_ptr and yaml_read_ptr
        DoubleIntegralPlanner *dip_ptr[6];
        YAML_READ *yaml_read_ptr;

        // Yaml directory and trajectory constraint
        // traj_constraint[0].name : PAN
        // traj_constraint[1].name : LIFT
        string yaml_dir;
        TRJ_constraint traj_constraint[2];

        // Mode value subscriber
        ros::Subscriber nh_mode_subscriber;

        // Actual value subscriber
        ros::Subscriber nh_actual_subscriber;

        // Twist subscriber (WHEEL velocity)
        ros::Subscriber nh_twist_subscriber;

        // Publisher of PAN, LIFT, WHEEL
        ros::Publisher nh_motors_publisher;

        // Publisher of dxl
        ros::Publisher nh_dxl_publisher;



        /**
         * Desired position of LIFT, PAN, and STEERING
         * Desired velocity of WHEEL
        */

        double des_pos_LIFT[NUM_LIFT];
        double des_vel_LIFT[NUM_LIFT];

        double des_pos_PAN[NUM_PAN];
        double des_vel_PAN[NUM_PAN];

        double des_pos_STEERING[NUM_DXL];
        double des_vel_WHEEL[NUM_WHEEL];

        // Goal position
        double goal_pos[3];

        double t_get_goal;
        double t_traj;

        /**
         * Target array
         * target_LIFT: position            (INC)           4,096 CPT --> 1 REV = 4 * 4,096 * N_LIFT
         * target_PAN: position             (INC)           1,024 CPT --> 1 REV = 4 * 1,024 * N_PAN 
         * target_WHEEL: velocity           (RPM)
         * target_dxl: extended position    (32 BITs)       0 deg = 2,048, 180 deg = 4,096, -180 deg = 0 
        */

        int32_t target_LIFT[NUM_LIFT];
        int32_t target_PAN[NUM_PAN];
        int32_t target_WHEEL[NUM_WHEEL];
        int32_t target_dxl[NUM_DXL];

        uint8_t mode_value;

        TRJ_DATA traj_data;

};


#endif