#include <iostream>
#include <cstdlib>

#include <Eigen/Dense>

#include "problem.h"


int main()
{
	Eigen::Vector2i numdim_output_bounds;
	numdim_output_bounds << 2, 2;
	Eigen::Vector2i num_integrators_bounds;
	num_integrators_bounds << 2, 2;

	Eigen::Vector4d Y_max;
	Y_max << 0, 10, 0, 10;

	Eigen::Vector4d U_max;
	U_max << -1, 1, -1, 1;

	Eigen::Vector2d period_bounds( 0.05, 0.1 );

	srand(time(0));
	Problem *prob = Problem::random( numdim_output_bounds,
									 num_integrators_bounds,
									 Y_max, U_max, 2, 1, period_bounds );
	std::cout << *prob << std::endl;

	prob->to_formula( std::cerr );
	std::cerr << std::endl;


	delete prob;
	return 0;
}
