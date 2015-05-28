#ifndef PROBLEM_H
#define PROBLEM_H

#include <vector>
#include <list>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstdlib>

#include <Eigen/Dense>

#include "polytope.h"


/** Representation of problem instances.

	\ingroup integrator_chains */
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

	/** Output description of problem in JSON to a stream */
	friend std::ostream & operator<<( std::ostream &out, const Problem &prob );

	/** Get description of problem in JSON. */
	std::string dumpJSON() const;

	/** Generate a random problem instance.

	   \param numdim_output_bounds range of integers from which the dimension of
	   the output space will be chosen.

	   \param highest_order_deriv_bounds range of integers from which the order
	   of derivation (i.e., the number of integrators in the ODE defining the
	   system) will be chosen.

	   \param Y_box the compact subset of the output space in which the
	   trajectory must remain, defined using the same format as for
	   Polytope::box(). Unlike most other parameters of this method, \p Y_box is
	   not a definition of the support of  a probability density
	   function. However, the number of elements in it that are actually used
	   depends on the randomly chosen number of dimensions of the output space.

	   \param U_box analogous to \p Y_box but for the input space.

	   \param number_goals_bounds range of integers from which the number of
	   goal polytopes in the output space will be chosen.

	   \param number_obstacles_bounds analogous to \p number_goals_bounds but
	   for obstacles.

	   \param period_bounds range from which is uniformly randomly chosen the
	   constant period by which the original system is discretized.

	   \param Xinit_bounds (optional) the box from which the initial state will
	   be chosen. It is assumed to be consistent with \p Y_box. Bounds for
	   missing dimensions are assumed to be [0,0], i.e., Xinit is assigned 0.0
	   at the corresponding indices. E.g., if Xinit_bounds has size 0 or is not
	   given, then Xinit is the origin of the state space.

	   N.B., several parameters depend on each other in terms of consistency.
	   E.g., if \p numdim_output_bounds = [2, 3], then Y_box must have length of
	   at least 6 (cf. documentation for Polytope::box()).

	   This method relies on rand() for pseudo-randomness and assumes that
	   something else has seeded the RNG. */
	static Problem * random( const Eigen::Vector2i &numdim_output_bounds,
							 const Eigen::Vector2i &highest_order_deriv_bounds,
							 const Eigen::VectorXd &Y_box,
							 const Eigen::VectorXd &U_box,
							 const Eigen::Vector2i &number_goals_bounds,
							 const Eigen::Vector2i &number_obstacles_bounds,
							 const Eigen::Vector2d &period_bounds,
							 const Eigen::VectorXd &Xinit_bounds = Eigen::VectorXd() );

	int get_numdim_output() const
	{ return numdim_output; }
	int get_highest_order_deriv() const
	{ return highest_order_deriv; }

private:
	int numdim_output;
	int highest_order_deriv;
	static std::string int_to_str( int x, int zero_padding=0 );
};

std::string Problem::int_to_str( int x, int zero_padding )
{
	std::ostringstream out;
	if (zero_padding > 0)
		out << std::setfill('0') << std::setw( zero_padding );
	out << x;
	return out.str();
}

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

std::string Problem::dumpJSON() const
{
	std::ostringstream out;
	out << *this;
	return out.str();
}


Problem * Problem::random( const Eigen::Vector2i &numdim_output_bounds,
						   const Eigen::Vector2i &highest_order_deriv_bounds,
						   const Eigen::VectorXd &Y_box,
						   const Eigen::VectorXd &U_box,
						   const Eigen::Vector2i &number_goals_bounds,
						   const Eigen::Vector2i &number_obstacles_bounds,
						   const Eigen::Vector2d &period_bounds,
						   const Eigen::VectorXd &Xinit_bounds )
{
	assert( numdim_output_bounds(0) >= 1
			&& numdim_output_bounds(0) <= numdim_output_bounds(1)
			&& highest_order_deriv_bounds(0) >= 1
			&& highest_order_deriv_bounds(0) <= highest_order_deriv_bounds(1)
			&& number_goals_bounds(0) >= 0
			&& number_goals_bounds(0) <= number_goals_bounds(1)
			&& number_obstacles_bounds(0) >= 0
			&& number_obstacles_bounds(0) <= number_obstacles_bounds(1)
			&& Y_box.size() >= 2*numdim_output_bounds(1)
			&& U_box.size() >= 2*numdim_output_bounds(1)
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

	int number_goals = number_goals_bounds(0);
	if (number_goals_bounds(1) != number_goals_bounds(0))
		number_goals += (rand()
						 % (1+number_goals_bounds(1)
							- number_goals_bounds(0)));

	int number_obstacles = number_obstacles_bounds(0);
	if (number_obstacles_bounds(1) != number_obstacles_bounds(0))
		number_obstacles += (rand()
						 % (1+number_obstacles_bounds(1)
							- number_obstacles_bounds(0)));

	prob->Y = Polytope::box( Y_box.head( 2*prob->numdim_output ) );
	prob->U = Polytope::box( U_box.head( 2*prob->numdim_output ) );

	prob->Xinit.setZero( prob->numdim_output*prob->highest_order_deriv );
	int largest_dim = prob->numdim_output*prob->highest_order_deriv;
	if (largest_dim > Xinit_bounds.size()/2)
		largest_dim = Xinit_bounds.size()/2;
	for (i = 0; i < largest_dim; i++) {
		assert( Xinit_bounds(2*i+1) >= Xinit_bounds(2*i) );
		prob->Xinit(i) = Xinit_bounds(2*i);
		if (Xinit_bounds(2*i+1) != Xinit_bounds(2*i))
			prob->Xinit(i) += (double(rand())/RAND_MAX)*(Xinit_bounds(2*i+1) - Xinit_bounds(2*i));
	}

	Eigen::VectorXd box_bounds(2*prob->numdim_output);
	prob->goals.resize( number_goals );
	for (i = 0; i < number_goals; i++) {
		for (j = 0; j < prob->numdim_output; j++) {
			box_bounds(2*j) = Y_box(2*j)
				+ (double(rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
			box_bounds(2*j+1) = Y_box(2*j)
				+ (double(rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
			if (box_bounds(2*j+1) < box_bounds(2*j)) {
				double tmp = box_bounds(2*j+1);
				box_bounds(2*j+1) = box_bounds(2*j);
				box_bounds(2*j) = tmp;
			}
		}
		prob->goals[i] = LabeledPolytope::box( box_bounds,
											   std::string("goal_") + int_to_str(i) );
	}
	prob->obstacles.resize( number_obstacles );
	for (i = 0; i < number_obstacles; i++) {
		for (j = 0; j < prob->numdim_output; j++) {
			box_bounds(2*j) = Y_box(2*j)
				+ (double(rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
			box_bounds(2*j+1) = Y_box(2*j)
				+ (double(rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
			if (box_bounds(2*j+1) < box_bounds(2*j)) {
				double tmp = box_bounds(2*j+1);
				box_bounds(2*j+1) = box_bounds(2*j);
				box_bounds(2*j) = tmp;
			}
		}
		prob->obstacles[i] = LabeledPolytope::box( box_bounds,
												   std::string("obstacle_") + int_to_str(i) );
	}

	prob->period = period_bounds(0);
	if (period_bounds(1) != period_bounds(0))
		prob->period += double(rand())/RAND_MAX*(period_bounds(1) - period_bounds(0));
	
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
class Labeler {
public:
	Labeler();

	/* importProblem() is called as part of instantiation. */
	Labeler( const Problem &probinstance );

	bool importProblem( const Problem &probinstance, bool check_collisions=true );
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
	importProblem( probinstance, false );
}

bool Labeler::importProblem( const Problem &probinstance, bool check_collisions )
{
	int i, j;
	bool no_collisions = true;
	for (i = 0; i < probinstance.goals.size(); i++) {
		if (!check_collisions) {
			regions.push_back( probinstance.goals[i] );
		} else {
			for (j = 0; j < regions.size(); j++) {
				if (regions[j]->label == probinstance.goals[i]->label) {
					std::cerr << "WARNING: Labeler found a collision while importing from Problem object." << std::endl;
					no_collisions = false;
					break;
				}
			}
			if (j >= regions.size())
				regions.push_back( probinstance.goals[i] );
		}
	}
	for (i = 0; i < probinstance.obstacles.size(); i++) {
		if (!check_collisions) {
			regions.push_back( probinstance.obstacles[i] );
		} else {
			for (j = 0; j < regions.size(); j++) {
				if (regions[j]->label == probinstance.obstacles[i]->label) {
					std::cerr << "WARNING: Labeler found a collision while importing from Problem object." << std::endl;
					no_collisions = false;
					break;
				}
			}
			if (j >= regions.size())
				regions.push_back( probinstance.obstacles[i] );
		}
	}
	return no_collisions;
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
