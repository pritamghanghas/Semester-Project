#ifndef WAYPOINTS_PARSER_H
#define WAYPOINTS_PARSER_H

#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

class WaypointsParser
{
public:
    WaypointsParser(std::string input_file_path_, std::vector<Eigen::Vector3d> & waypoints_);

};

#endif // WAYPOINTS_PARSER_H
