#include <traj_gen/traj_gen.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_gen_node");
    Traj_Generator traj_gen;

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        traj_gen.move_motors();

        loop_rate.sleep();

    }



    return 0;
}