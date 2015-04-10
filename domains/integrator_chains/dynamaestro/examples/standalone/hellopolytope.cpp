#include <iostream>

#include <Eigen/Dense>

#include "polytope.h"


int main()
{
	Eigen::Vector4d bounds;
	bounds << 0, 1,
		      0, 1;
	Polytope *square = Polytope::box( bounds );

	std::cout << *square << std::endl;

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
