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
    RoadNetwork rnd( infile );
    std::cout << rnd << std::endl;

    return 0;
}
