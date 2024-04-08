#ifndef YAML_READ_HPP
#define YAML_READ_HPP

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cout;
using std::endl;
using std::cerr;

typedef struct TRJ_constraint_
{
    string name;
    double a_max;
    double v_max;
} TRJ_constraint;

class YAML_READ{
    
    public:

        YAML_READ();
        YAML_READ(string &file_name);

        void YamlLoadFile();

        TRJ_constraint traj_constraint[2];

    private:

        YAML::Node config;
        string file_name;

};

#endif