#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "roadnet.hpp"


int main()
{
    Eigen::Vector3d transform( 0.0, 0.0, 0.0 );
    RoadNetwork rd( 2.0, transform, 2, 3 );
    std::cout << rd << std::endl;
    return 0;
}
