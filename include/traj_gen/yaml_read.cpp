#include "yaml_read.hpp"

YAML_READ::YAML_READ()
{
    file_name = "/home/kay/Documents/cpp_test/yalm_study/config/test.yaml";
    cout<<"Default setup:"<<file_name<<endl;

}

YAML_READ::YAML_READ(string & file_name_)
{
    file_name = file_name_;
    cout<<"Yaml file directory path: ";
    cout<<file_name<<endl;
}

void YAML_READ::YamlLoadFile()
{

    try{

        YAML::Node config = YAML::LoadFile(file_name);

        cout<<"Yaml node is generated."<<endl;

        for(auto it:config["motors"])
        {
            if(it["name"].as<string>() == "PAN")
            {
                traj_constraint[0].name = it["name"].as<string>();
                traj_constraint[0].a_max = it["a_max"].as<double>();
                traj_constraint[0].v_max = it["v_max"].as<double>();
            }
            else
            {
                traj_constraint[1].name = it["name"].as<string>();
                traj_constraint[1].a_max = it["a_max"].as<double>();
                traj_constraint[1].v_max = it["v_max"].as<double>();
            }
        }

        // for(int i = 0; i < 2; i++)
        // {
        //     cout<<traj_constraint[i].name<<endl;
        //     cout<<traj_constraint[i].a_max<<endl;
        //     cout<<traj_constraint[i].v_max<<endl;
        // }


    }
    catch(const YAML::BadFile& e)
    {
        cerr<<e.msg<<endl;
    }
    catch(const YAML::ParserException& e)
    {
        cerr<<e.msg<<endl;
    }

}