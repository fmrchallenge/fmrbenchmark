#ifndef DUBINS_TRAFFIC_PROBLEM_H
#define DUBINS_TRAFFIC_PROBLEM_H


#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "roadnet.hpp"


namespace dubins_traffic {


/** Representation of problem instances.

    \ingroup dubins_traffic */
class Problem {
public:
    double intersection_radius;
    std::vector<size_t> goals;
    RoadNetwork rnd;

    Problem( const RoadNetwork &rnd_ );
    Problem( const Problem &to_be_copied );
    ~Problem() {}

    static Problem random( const RoadNetwork &rnd_,
                           const Eigen::Vector2i &number_goals_bounds );

    /** Create formula using SPIN LTL syntax <http://spinroot.com/spin/Man/ltl.html>.
       Support for other syntax is coming soon. */
    void to_formula( std::ostream &out ) const;

    /** Output description of problem in JSON to a stream */
    friend std::ostream & operator<<( std::ostream &out, const Problem &prob );

    /** Get description of problem in JSON. */
    std::string dumpJSON() const;
};

Problem::Problem( const RoadNetwork &rnd_ )
    : rnd(rnd_)
{
    intersection_radius = 0.0;
}

Problem::Problem( const Problem &to_be_copied )
    : intersection_radius( to_be_copied.intersection_radius ),
      goals( to_be_copied.goals ),
      rnd( to_be_copied.rnd )
{
    assert( intersection_radius > 0 );
}

Problem Problem::random( const RoadNetwork &rnd_,
                         const Eigen::Vector2i &number_goals_bounds )
{
    assert( number_goals_bounds(0) >= 0
            && number_goals_bounds(0) <= number_goals_bounds(1) );

    Problem pinstance( rnd_ );
    pinstance.intersection_radius = 1.0;

    int number_goals = number_goals_bounds(0);
    if (number_goals_bounds(1) != number_goals_bounds(0))
        number_goals += (std::rand()
                         % (1+number_goals_bounds(1)
                            - number_goals_bounds(0)));

    for (size_t goal_idx = 0; goal_idx < number_goals; goal_idx++)
        pinstance.goals.push_back( std::rand() % rnd_.number_of_intersections() );

    return pinstance;
}

void Problem::to_formula( std::ostream &out ) const
{
    if (goals.size() > 0) {
        for (size_t i = 0; i < goals.size(); i++) {
            if (i > 0)
                out << " && ";
            out << "([]<> ego_" << rnd.get_intersection_str( goals[i] ) << ")";
        }
    }
}

std::ostream & operator<<( std::ostream &out, const Problem &prob )
{
    out << "{" << std::endl << "\"version\": " << 0 << "," << std::endl;
    out << "\"goals\": [" << std::endl;
    for (int i = 0; i < prob.goals.size(); i++) {
        if (i > 0)
            out << ", ";
        out << "\"ego_" << prob.rnd.get_intersection_str( prob.goals[i] ) << "\"";
        out << std::endl;
    }
    out << "]";
    out << std::endl << "}";
    return out;
}

std::string Problem::dumpJSON() const
{
    std::ostringstream out;
    out << *this;
    return out.str();
}


} // namespace dubins_traffic


#endif
