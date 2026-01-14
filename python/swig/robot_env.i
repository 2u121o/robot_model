%module robot_env_cpp

%{
#include "RobotEnv.hpp"
%}

%include "std_string.i"
%include "std_vector.i"

%template(DoubleVector) std::vector<double>;

%include "RobotEnv.hpp"
