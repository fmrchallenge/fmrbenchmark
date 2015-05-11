#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include <list>
#include <string>
#include <cstdlib>

#include <Eigen/Dense>

#include "polytope.h"


/** \ingroup integrator_chains */
class Problem {
public:
	Eigen::VectorXd Xinit;
	Polytope *Y;
	Polytope *U;
	std::vector<LabeledPolytope *> goals;
	std::vector<LabeledPolytope *> obstacles;

	double period;

	Problem();
	~Problem();

	bool is_consistent() const;

	/** Create formula using SPIN LTL syntax <http://spinroot.com/spin/Man/ltl.html>.
	   Support for other syntax is coming soon. */
	void to_formula( std::ostream &out ) const;

	/* Change the number of dimensions of output or the order of the
	   differential operator (i.e., the number of integrators). Return True on
	   success, False otherwise, such as if one of the numbers is negative.

	   This method only changes the internal values corresponding to
	   numdim_output and highest_order_deriv but not any of the other parts of
	   this problem instance. As such, it is possible to have an inconsistent
	   instance after this call. (Try is_consistent() to check.) */
	bool change_order( int new_numdim_output , int new_highest_order_deriv );

	/** Output description of problem in JSON */
	friend std::ostream & operator<<( std::ostream &out, const Problem &prob );

	/** Generate a random problem instance.

	   \param numdim_output_bounds range of integers from which the dimension of
	   the output space will be chosen.

	   \param highest_order_deriv_bounds range of integers from which the order
	   of derivation (i.e., the number of integrators in the ODE defining the
	   system) will be chosen.

	   \param Y_max rectangle that is the maximum possible subset of the output
	   space that is permitted. It is of the same format as for Polytope::box().

	   \param U_max analogous to \p Y_max but for the input space.

	   \param period_bounds range from which is uniformly randomly chosen the
	   constant period by which the original system is discretized.

	   N.B., several parameters depend on each other in terms of consistency.
	   E.g., if \p numdim_output_bounds = [2, 3], then Y_max must have length of
	   at least 6 (cf. documentation for Polytope::box()).

	   This method relies on rand() for pseudo-randomness and assumes that
	   something else has seeded the RNG. */
	static Problem * random( const Eigen::Vector2i &numdim_output_bounds,
							 const Eigen::Vector2i &highest_order_deriv_bounds,
							 const Eigen::VectorXd &Y_max,
							 const Eigen::VectorXd &U_max,
							 int max_number_goals, int max_number_obstacles,
							 const Eigen::Vector2d &period_bounds );

	int get_numdim_output() const
	{ return numdim_output; }
	int get_highest_order_deriv() const
	{ return highest_order_deriv; }

private:
	int numdim_output;
	int highest_order_deriv;
};

Problem::Problem()
	: numdim_output(0), highest_order_deriv(0), Y(NULL), U(NULL)
{ }

Problem::~Problem()
{
	if (Y)
		delete Y;
	if (U)
		delete U;
	for (int i = 0; i < goals.size(); i++)
		delete goals[i];
	for (int i = 0; i < obstacles.size(); i++)
		delete obstacles[i];
}

bool Problem::change_order( int new_numdim_output , int new_highest_order_deriv )
{
	if (new_numdim_output < 0 || new_highest_order_deriv < 0)
		return false;

	numdim_output = new_numdim_output;
	highest_order_deriv = new_highest_order_deriv;

	return true;
}

std::ostream & operator<<( std::ostream &out, const Problem &prob )
{
	out << "{" << std::endl << "\"version\": " << 0 << "," << std::endl;
	out << "\"Xinit\": [";
	for (int i = 0; i < prob.Xinit.size(); i++) {
		if (i > 0)
			out << ", ";
		out << prob.Xinit(i);
	}
	out << "]," << std::endl;
	out << "\"goals\": [" << std::endl;
	for (int i = 0; i < prob.goals.size(); i++) {
		if (i > 0)
			out << ", ";
		out << *prob.goals[i];
		out << std::endl;
	}
	out << "]," << std::endl;
	out << "\"obstacles\": [";
	for (int i = 0; i < prob.obstacles.size(); i++) {
		if (i > 0)
			out << ", ";
		out << *prob.obstacles[i];
		out << std::endl;
	}
	out << "]," << std::endl;
	out << "\"Y\": ";
	out << *prob.Y;
	out << "," << std::endl;
	out << "\"U\": ";
	out << *prob.U;
	out << "," << std::endl;
	out << "\"period\": " << prob.period;
	out << std::endl << " }";
	return out;
}


Problem * Problem::random( const Eigen::Vector2i &numdim_output_bounds,
						   const Eigen::Vector2i &highest_order_deriv_bounds,
						   const Eigen::VectorXd &Y_max,
						   const Eigen::VectorXd &U_max,
						   int max_number_goals, int max_number_obstacles,
						   const Eigen::Vector2d &period_bounds )
{
	assert( numdim_output_bounds(0) >= 1
			&& numdim_output_bounds(0) <= numdim_output_bounds(1)
			&& highest_order_deriv_bounds(0) >= 1
			&& highest_order_deriv_bounds(0) <= highest_order_deriv_bounds(1)
			&& max_number_goals >= 0 && max_number_obstacles >= 0
			&& Y_max.size() >= 2*numdim_output_bounds(1)
			&& U_max.size() >= 2*numdim_output_bounds(1)
			&& period_bounds(0) >= 0 && period_bounds(1) >= period_bounds(0) );

	int i, j;
	Problem *prob = new Problem;

	prob->numdim_output = numdim_output_bounds(0);
	if (numdim_output_bounds(1) != numdim_output_bounds(0))
		prob->numdim_output += (rand()
								% (1+numdim_output_bounds(1)
								   - numdim_output_bounds(0)));

	prob->highest_order_deriv = highest_order_deriv_bounds(0);
	if (highest_order_deriv_bounds(1) != highest_order_deriv_bounds(0))
		prob->highest_order_deriv += (rand()
									  % (1+highest_order_deriv_bounds(1)
										 - highest_order_deriv_bounds(0)));

	int number_goals = (rand() % max_number_goals) + 1;
	int number_obstacles = (rand() % max_number_obstacles) + 1;

	Eigen::VectorXd Y_bounds(2*prob->numdim_output);
	for (i = 0; i < prob->numdim_output; i++) {
		Y_bounds(2*i) = Y_max(2*i)
			+ (double(rand())/RAND_MAX)*(Y_max(2*i+1) - Y_max(2*i));
		Y_bounds(2*i+1) = Y_max(2*i)
			+ (double(rand())/RAND_MAX)*(Y_max(2*i+1) - Y_max(2*i));
		if (Y_bounds(2*i+1) < Y_bounds(2*i)) {
			double tmp = Y_bounds(2*i+1);
			Y_bounds(2*i+1) = Y_bounds(2*i);
			Y_bounds(2*i) = tmp;
		}
	}
	prob->Y = Polytope::box( Y_bounds );

	Eigen::VectorXd U_bounds(2*prob->numdim_output);
	for (i = 0; i < prob->numdim_output; i++) {
		U_bounds(2*i) = U_max(2*i)
			+ (double(rand())/RAND_MAX)*(U_max(2*i+1) - U_max(2*i));
		U_bounds(2*i+1) = U_max(2*i)
			+ (double(rand())/RAND_MAX)*(U_max(2*i+1) - U_max(2*i));
		if (U_bounds(2*i+1) < U_bounds(2*i)) {
			double tmp = U_bounds(2*i+1);
			U_bounds(2*i+1) = U_bounds(2*i);
			U_bounds(2*i) = tmp;
		}
	}
	prob->U = Polytope::box( U_bounds );

	prob->Xinit.setZero( prob->numdim_output*prob->highest_order_deriv );
	for (i = 0; i < prob->numdim_output; i++) {
		prob->Xinit(i) = Y_bounds(2*i)
			+ (double(rand())/RAND_MAX)*(Y_bounds(2*i+1) - Y_bounds(2*i));
	}

	Eigen::VectorXd box_bounds(2*prob->numdim_output);
	prob->goals.resize( number_goals );
	for (i = 0; i < number_goals; i++) {
		for (j = 0; j < prob->numdim_output; j++) {
			box_bounds(2*j) = Y_bounds(2*j)
				+ (double(rand())/RAND_MAX)*(Y_bounds(2*j+1) - Y_bounds(2*j));
			box_bounds(2*j+1) = Y_bounds(2*j)
				+ (double(rand())/RAND_MAX)*(Y_bounds(2*j+1) - Y_bounds(2*j));
			if (box_bounds(2*j+1) < box_bounds(2*j)) {
				double tmp = box_bounds(2*j+1);
				box_bounds(2*j+1) = box_bounds(2*j);
				box_bounds(2*j) = tmp;
			}
		}
		prob->goals[i] = LabeledPolytope::box( box_bounds,
											   std::string("goal_") + char(0x30+i) );
	}
	prob->obstacles.resize( number_obstacles );
	for (i = 0; i < number_obstacles; i++) {
		for (j = 0; j < prob->numdim_output; j++) {
			box_bounds(2*j) = Y_bounds(2*j)
				+ (double(rand())/RAND_MAX)*(Y_bounds(2*j+1) - Y_bounds(2*j));
			box_bounds(2*j+1) = Y_bounds(2*j)
				+ (double(rand())/RAND_MAX)*(Y_bounds(2*j+1) - Y_bounds(2*j));
			if (box_bounds(2*j+1) < box_bounds(2*j)) {
				double tmp = box_bounds(2*j+1);
				box_bounds(2*j+1) = box_bounds(2*j);
				box_bounds(2*j) = tmp;
			}
		}
		prob->obstacles[i] = LabeledPolytope::box( box_bounds,
												   std::string("obstacle_") + char(0x30+i) );
	}

	prob->period = period_bounds(0)
		+ double(rand())/RAND_MAX*(period_bounds(1) - period_bounds(0));
	
	return prob;
}

void Problem::to_formula( std::ostream &out ) const
{
	if (obstacles.size() > 0) {
		out << "[]!(";
		for (int i = 0; i < obstacles.size(); i++) {
			if (i > 0)
				out << " || ";
			out << obstacles[i]->label;
		}
		out << ")";
	}
	if (goals.size() > 0) {
		if (obstacles.size() > 0)
			out << " && ";

		for (int i = 0; i < goals.size(); i++) {
			if (i > 0)
				out << " && ";
			out << "([]<> " << goals[i]->label << ")";
		}
	}
}


/** \ingroup integrator_chains */
class Labeler
{
public:
	Labeler();
	Labeler( const Problem &probinstance );

	std::list<std::string> get_label( const Eigen::VectorXd &X );
	void clear();

private:
	std::vector<LabeledPolytope *> regions;
};

Labeler::Labeler()
	: regions()
{ }

Labeler::Labeler( const Problem &probinstance )
{
	int i, j;
	for (i = 0; i < probinstance.goals.size(); i++) {
		for (j = 0; j < regions.size(); j++) {
			if (regions[j]->label == probinstance.goals[i]->label) {
				std::cerr << "WARNING: Labeler found a collision while importing from Problem object." << std::endl;
				break;
			}
		}
		if (j >= regions.size())
			regions.push_back( probinstance.goals[i] );
	}
	for (i = 0; i < probinstance.obstacles.size(); i++) {
		for (j = 0; j < regions.size(); j++) {
			if (regions[j]->label == probinstance.obstacles[i]->label) {
				std::cerr << "WARNING: Labeler found a collision while importing from Problem object." << std::endl;
				break;
			}
		}
		if (j >= regions.size())
			regions.push_back( probinstance.obstacles[i] );
	}
}

std::list<std::string> Labeler::get_label( const Eigen::VectorXd &X )
{
	std::list<std::string> label;
	for (int i = 0; i < regions.size(); i++) {
		if (regions[i]->is_in( X ))
			label.push_back( regions[i]->label );
	}
	return label;
}

void Labeler::clear()
{
	regions.clear();
}


#endif  // ifndef PROBLEM_H
