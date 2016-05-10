#include <fstream>
#include <iostream>

#include "roadnet.hpp"


int main( int argc, char *argv[] )
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " FILE" << std::endl;
        return 1;
    }

    std::string buf;
    std::ifstream infile( argv[1] );
    if (!infile.is_open()) {
        std::cerr << "Failed to open " << argv[1] << std::endl;
        return 1;
    }

    dubins_traffic::RoadNetwork rnd( infile );
    std::cout << rnd << std::endl;

    return 0;
}
