#include "traj_gen.hpp"

Traj_Generator::Traj_Generator()
{
    cout<<"Constructor is called.\n";

    for(int i = 0; i < 3; i++)
    {
        des_pos_LIFT[i] = 0;
        des_vel_LIFT[i] = 0;

        des_pos_PAN[i] = 0;
        des_vel_LIFT[i] = 0;

        des_pos_STEERING[i] = 0;
        des_vel_WHEEL[i] = 0;

        goal_pos[i] = 0;
    }

    for(int i = 0; i < 6; i++)
    {
        traj_data.a_curr[i] = 0;
        traj_data.v_curr[i] = 0;
        traj_data.p_curr[i] = 0;
    }

    mode_value = 255;

    get_yaml_dir();

    // Allocate dynamic memory for yaml_read
    // and call the corresponding constructor.
    yaml_read_ptr = new YAML_READ(yaml_dir);

    constraint_setup();

    nh_mode_subscriber = nh_.subscribe("/mode_val",
                                        10,
                                        &Traj_Generator::callbackModeVal,
                                        this);
    


}

void Traj_Generator::get_yaml_dir()
{
    nh_.getParam("yaml_config", yaml_dir);
}


void Traj_Generator::constraint_setup()
{
    yaml_read_ptr->YamlLoadFile();
    for(int i = 0; i < 2; i++)
        traj_constraint[i] = yaml_read_ptr->traj_constraint[i];
    
    // Get trajectory constraints
    for(int i = 0; i < 2; i++)
    {
        cout<<"Name: "<<traj_constraint[i].name<<endl;
        cout<<"a_max: "<<traj_constraint[i].a_max<<endl;
        cout<<"v_max: "<<traj_constraint[i].v_max<<endl;
    }
    // Allocate dynamic memory for dip
    // and call the corresponding constructor.
    for(int i = 0; i < 6; i++)
        dip_ptr[i] = new DoubleIntegralPlanner(
            traj_constraint[i/3].a_max, 
            traj_constraint[i/3].v_max);

}

void Traj_Generator::callbackModeVal(const mode::ConstPtr& mode_ref)
{
    for(int i = 0; i < 3; i++)
        goal_pos[i] = mode_ref->goal_pos[i];

    mode_value = mode_ref->mode_val;

    if(mode_value == 1)
    {
        set_DXL_BEFORE_LIFT();
    }
    else if(mode_value == 2)
    {
        set_LIFT();
    }
    else if(mode_value == 3)
    {
        set_DXL_BEFORE_PAN();
    }
    else if(mode_value == 4)
    {
        set_PAN();
    }

}

void Traj_Generator::callbackActual(const actual::ConstPtr& actual_ref)
{

}

void Traj_Generator::callbackTwist(const Twist::ConstPtr& twist_ref)
{

}

void Traj_Generator::set_DXL_BEFORE_LIFT()
{
    for(int i = 0; i < 3; i++)
        target_dxl[i] = 2048;

}

void Traj_Generator::set_LIFT()
{
    for(int i = 0; i < 3; i++)
        setGoalPos(goal_pos[i], i+3);
    
    t_get_goal = ros::Time::now().toSec();

}

void Traj_Generator::set_DXL_BEFORE_PAN()
{

}

void Traj_Generator::set_PAN()
{
    for(int i = 0; i < 3; i++)
        setGoalPos(goal_pos[i], i);

    t_get_goal = ros::Time::now().toSec();
}

void Traj_Generator::move_motors()
{
    if(mode_value == 1)
    {

    }
    else if(mode_value == 2)
    {
        t_traj = ros::Time::now().toSec() - t_get_goal - 2;
        cout<<"Time: "<<t_traj<<endl;
        move_LIFT_motors();
    }
    else if(mode_value == 3)
    {

    }
    else if(mode_value == 4)
    {
        t_traj = ros::Time::now().toSec() - t_get_goal - 2;
        cout<<"Time: "<<t_traj<<endl;
        move_PAN_motors();
    }
}

void Traj_Generator::move_PAN_motors()
{
    cout<<"**************************"<<endl;
    for(int i = 0; i < 3; i++)
        getTraj(traj_data, i, t_traj);
}

void Traj_Generator::move_LIFT_motors()
{
    for(int i = 0; i < 3; i++)
        getTraj(traj_data, i+3, t_traj);
}

void Traj_Generator::setGoalPos(double goal_pos, int idx)
{
    dip_ptr[idx]->setGoalPos(goal_pos);
    dip_ptr[idx]->TimeCalc();
}

void Traj_Generator::getTraj(TRJ_DATA &traj_ref, int idx, double time)
{
    dip_ptr[idx]->goalTraj(time);
    traj_ref.p_curr[idx] = dip_ptr[idx]->getPos();
    traj_ref.v_curr[idx] = dip_ptr[idx]->getVel();
    cout<<"Motor ["<<idx<<"] : "<<traj_ref.p_curr[idx]<<endl;
}

Traj_Generator::~Traj_Generator()
{
    // Delete the allocated dynamic memory for dip
    for(int i = 0; i < 6; i++)
        delete dip_ptr[i];

    // Delete the allocated dynamic memory for yaml_read
    delete yaml_read_ptr;

    cout<<"Destructor is called."<<endl;
}