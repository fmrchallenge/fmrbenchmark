#include <iostream>

#include <Eigen/Dense>

#include "polytope.hpp"

using namespace integrator_chains;


int main()
{
    Eigen::Vector4d bounds;
    bounds << 0, 1,
              0, 1;
    LabeledPolytope *square = LabeledPolytope::box( bounds, "square" );

    bounds << 0.5, 1.5,
              0.2, 2;
    Polytope *rect1 = Polytope::box( bounds );
    Polytope rect2 = *rect1 & *square;

    std::cout << *square << std::endl;
    std::cout << *rect1 << std::endl;
    std::cout << rect2 << std::endl;

    Eigen::Vector2d X;
    X << 0.3, 0.5;
    do {
        std::cout << "The vector (" << X(0) << ", " << X(1) << ")";
        if (square->is_in( X )) {
            std::cout << " is in ";
        } else {
            std::cout << " is not in ";
        }
        std::cout << "[0,1]^2" << std::endl;
        X(0) += 0.4;
    } while (X(0) < 1.5);

    delete square;
    return 0;
}
