#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "roadnet.hpp"


int main()
{
    Eigen::Vector3d transform( 1.0, 0.0, 1.6 );
    dubins_traffic::RoadNetwork rd( 2.0, transform, 2, 3 );
    std::cout << rd << std::endl;

    double x, y;
    rd.map_point( 0, 0, x, y );
    std::cerr << "(0, 0) -> (" << x << ", " << y << ")" << std::endl;

    rd.map_point( 1, 2, x, y );
    std::cerr << "(1, 2) -> (" << x << ", " << y << ")" << std::endl;

    for (size_t idx = 0; idx < rd.number_of_segments(); idx++)
        std::cerr << rd.mapped_segment( idx ).transpose() << std::endl;

    return 0;
}
