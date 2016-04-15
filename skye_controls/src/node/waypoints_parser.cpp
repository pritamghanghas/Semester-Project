#include "waypoints_parser.h"

WaypointsParser::WaypointsParser(std::string input_file_path_, std::vector<Eigen::Vector3d> & waypoints_){
    std::cout << "path file: " << input_file_path_ << std::endl;

    // load all the files

    std::ifstream in(input_file_path_.c_str());
    std::cout << "STREAM OPENED" << std::endl;

    for(std::string line; std::getline(in, line);){
        std::cout << "Parsed line: " << line << std::endl;

        std::size_t found_point = line.find("Point");
        std::cout << "Found point: " << found_point << std::endl;


        //If there is no point skip the line
        if (found_point == std::string::npos) continue;
        // Here I have a good line
        std::size_t found_column = line.find(":");
        if (found_column == std::string::npos) continue; //if true the format is wrong
        std::cout << "found_column: " << found_column<< std::endl;

        char single_character = line[found_column];
        std::cout << "single_character: " << single_character << std::endl;
        int i = found_column;

        std::vector<double> single_waypoint;
        for (int j = 0; j<3; ++j){
            char vector_element[10] = {'\0'};
            int chars_found_ = 0;

            while(line[i] != ',' && i<line.size()) {

                char single_character = line[i];


                bool is_meaningful_ = std::isdigit(single_character) || single_character == '-' || single_character == '.';

                if (is_meaningful_) {
                    vector_element[chars_found_] = single_character;
                    std::cout << "meaningful_character: " << single_character << std::endl;
                    chars_found_++;
                }
                i++;
//                std::cout << "line size: " << line.size() << " | i: " << i << std::endl;

            }
            std::cout << "vector_element: " << vector_element << std::endl;
            single_waypoint.push_back( std::atof(vector_element) );
            i++;
        }
        Eigen::Vector3d final_vec_;
        final_vec_ << single_waypoint.at(0), single_waypoint.at(1), single_waypoint.at(2);
        waypoints_.push_back(final_vec_);
        std::cout << "final_vec_: " << final_vec_ << std::endl;


    }

}
