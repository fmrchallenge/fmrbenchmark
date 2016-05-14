#include <iostream>
#include <cstdlib>
#include <ctime>

#include <Eigen/Dense>

#include "problem.hpp"

using namespace integrator_chains;


int main()
{
    Eigen::Vector2i numdim_output_bounds( 2, 2 );
    Eigen::Vector2i num_integrators_bounds( 2, 2 );

    Eigen::Vector4d Y_box;
    Y_box << -1, 10, -2, 10;

    Eigen::Vector4d U_box;
    U_box << -1, 1, -1, 1;

    Eigen::Vector2d period_bounds( 0.05, 0.1 );

    Eigen::Vector2i number_goals_bounds( 1, 2 );
    Eigen::Vector2i number_obstacles_bounds( 0, 1 );

    std::srand( std::time( 0 ) );
    Problem *prob = Problem::random( numdim_output_bounds,
                                     num_integrators_bounds,
                                     Y_box, U_box,
                                     number_goals_bounds,
                                     number_obstacles_bounds,
                                     period_bounds );
    std::cout << *prob << std::endl;

    prob->to_formula( std::cerr );
    std::cerr << std::endl;


    delete prob;
    return 0;
}
