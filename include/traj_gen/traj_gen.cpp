#include "traj_gen.hpp"


Traj_Generator::Traj_Generator()
{
    cout<<"Constructor is called.\n";

    // Get Yaml directory from launch file.
    nh_.getParam("yaml_config", yaml_dir);

    for(int i = 0; i < 3; i++)
        init_pos[i] = 0;


    /**
     * Parameter setup
    */
    nh_.getParam("offset_pos0",offset_pos[0]);
    nh_.getParam("offset_pos1",offset_pos[1]);
    nh_.getParam("offset_pos2",offset_pos[2]);

    for(int i = 0; i < 3; i++)
    {
        cout<<"Offset pos["<<i<<"]: "<<offset_pos[i]<<endl;
        init_pos[i+3] = offset_pos[i];
    }

    for(int i = 0; i < 3; i++)
    {
        des_pos_STEERING[i] = 0;
        des_linear_vel_WHEEL[i] = 0;
        des_angular_vel_WHEEL[i] = 0;
        goal_pos[i] = 0;
    }

    for(int i = 0; i < 6; i++)
    {
        traj_data.a_curr[i] = 0;
        traj_data.v_curr[i] = 0;
        traj_data.p_curr[i] = 0;
        des_pos[i] = 0;
    }

    mode_value = 255;


    // Allocate dynamic memory for yaml_read
    // and call the corresponding constructor.
    yaml_read_ptr = new YAML_READ(yaml_dir);

    constraint_setup();

    wheel_vel_gen_ptr = new WHEEL_VEL_GEN();

    nh_mode_subscriber = nh_.subscribe("/mode_val",
                                        10,
                                        &Traj_Generator::callbackModeVal,
                                        this);

    nh_actual_subscriber = nh_.subscribe("/actual",
                                        10,
                                        &Traj_Generator::callbackActual,
                                        this);
    
    nh_motors_publisher = nh_.advertise<target>("/target", 10);

    nh_dxl_publisher = nh_.advertise<target_dxl>("/target_dxl",10);

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
    for(int i = 0; i < 3; i++)
    {
        actual_pos_LIFT[i] = actual_ref->act_LIFT_pos[i];
        pos_LIFT[i] = convert_actual2pos_LIFT(
            actual_pos_LIFT[i]);
    }


}

void Traj_Generator::callbackTwist(const Twist::ConstPtr& twist_ref)
{

}

void Traj_Generator::set_DXL_BEFORE_LIFT()
{
    for(int i = 0; i < 3; i++)
    {
        des_pos_STEERING[i] = 0.0;
        target_dxl_[i] = (int32_t)(des_pos_STEERING[i] * 4096.0/360.0 + 2048.0);
        std::cout<<"mode 1"<<std::endl;
    }
}

void Traj_Generator::set_LIFT()
{
    for(int i = 0; i < 3; i++)
        setGoalPos(goal_pos[i] - init_pos[i+3], i+3);
    
    t_get_goal = ros::Time::now().toSec();
}

void Traj_Generator::set_DXL_BEFORE_PAN()
{
    for(int i = 0; i < 3; i++)
    {
        des_pos_STEERING[i] = -90.0;
        target_dxl_[i] = (int32_t)(des_pos_STEERING[i] * 4096.0/360.0 + 2048.0);
    }
}

void Traj_Generator::set_PAN()
{
    for(int i = 0; i < 3; i++)
        setGoalPos(goal_pos[i] - init_pos[i], i);

    t_get_goal = ros::Time::now().toSec();
}

void Traj_Generator::move_motors()
{
    if(mode_value == 1)
    {
        target_dxl_msg.stamp = ros::Time::now();
        for(int i = 0; i < 3; i++)
            target_dxl_msg.target_dxl[i] = target_dxl_[i];
        publish_target_dxl();
    }
    else if(mode_value == 2)
    {
        t_traj = ros::Time::now().toSec() - t_get_goal - 2;
        cout<<"****************"<<endl;
        cout<<"Time: "<<t_traj<<endl;
        move_LIFT_motors();
        
        wheel_vel_gen_ptr->get_wheel_vel(
            des_linear_vel_WHEEL,
            des_angular_vel_WHEEL, 
            pos_LIFT, 
            traj_data.v_curr, 
            LIFT_WHEEL);
        
        publish_target();
    }
    else if(mode_value == 3)
    {
        target_dxl_msg.stamp = ros::Time::now();
        for(int i = 0; i < 3; i++)
            target_dxl_msg.target_dxl[i] = target_dxl_[i];
        publish_target_dxl();
    }
    else if(mode_value == 4)
    {
        t_traj = ros::Time::now().toSec() - t_get_goal - 2;
        cout<<"****************"<<endl;
        cout<<"Time: "<<t_traj<<endl;
        move_PAN_motors();

        wheel_vel_gen_ptr->get_wheel_vel(
        des_linear_vel_WHEEL,
        des_angular_vel_WHEEL, 
        pos_LIFT, 
        traj_data.v_curr, 
        PAN_WHEEL);
        
        publish_target();
    }
}

void Traj_Generator::publish_target()
{
    target_msg.stamp = ros::Time::now();
    for(int i = 0; i < 3; i++)
    {
        target_msg.target_PAN[i] = convert_deg2target_PAN(des_pos[i]);
        target_msg.target_LIFT[i] = -convert_deg2target_LIFT(des_pos[i+3]-offset_pos[i]);
        target_msg.target_WHEEL[i] = (int32_t) (des_angular_vel_WHEEL[i]*degps2RPM);
        cout<<"wheel vel: "<<des_angular_vel_WHEEL[i]<<"\t";
    }
    cout<<"\n";
    nh_motors_publisher.publish(target_msg);
}

void Traj_Generator::publish_target_dxl()
{
    target_dxl_msg.stamp = ros::Time::now();
    for(int i = 0; i < 3; i++)
    {
        target_dxl_msg.target_dxl[i] = convert_deg2target_DXL(des_pos_STEERING[i]);
    }
    nh_dxl_publisher.publish(target_dxl_msg);
}

void Traj_Generator::move_PAN_motors()
{
    for(int i = 0; i < 3; i++)
        getTraj(traj_data, i, t_traj);
    
    init_des_traj(0);

    for(int i = 0; i < 3; i++)
        des_pos[i+3] = init_pos[i+3];

    for(int i = 0 ; i < 6; i++)
        cout<<"Motor ["<<i<<"] : "<<des_pos[i]<<endl;
}

void Traj_Generator::move_LIFT_motors()
{
    for(int i = 0; i < 3; i++)
        getTraj(traj_data, i+3, t_traj);

    init_des_traj(3);

    for(int i = 0; i < 3; i++)
        des_pos[i] = init_pos[i];

    for(int i = 0 ; i < 6; i++)
        cout<<"Motor ["<<i<<"] : "<<des_pos[i]<<endl;
}

void Traj_Generator::init_des_traj(int offset)
{
    for(int i = 0; i < 3; i++)
    {
        if(t_traj > dip_ptr[i+offset]->getFinalTime() + 1)
        {
            init_pos[i+offset] = des_pos[i+offset];
        }
        else
        {
            des_pos[i+offset] = traj_data.p_curr[i+offset] + init_pos[i+offset];
        }
    }
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

}

Traj_Generator::~Traj_Generator()
{
    // Delete the allocated dynamic memory for dip
    for(int i = 0; i < 6; i++)
        delete dip_ptr[i];

    // Delete the allocated dynamic memory for yaml_read
    delete yaml_read_ptr;

    delete wheel_vel_gen_ptr;

    cout<<"Destructor is called."<<endl;
}
