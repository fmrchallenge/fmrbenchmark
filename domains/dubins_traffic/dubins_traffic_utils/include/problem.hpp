#ifndef DUBINS_TRAFFIC_PROBLEM_H
#define DUBINS_TRAFFIC_PROBLEM_H


#include <iostream>

#include "roadnet.hpp"


namespace dubins_traffic {


/** Representation of problem instances.

    \ingroup dubins_traffic */
class Problem {
public:
    double intersection_radius;
    std::vector<size_t> goals;
    RoadNetwork &rnd;

    Problem( RoadNetwork &rnd_ );
    ~Problem() {}

    /** Create formula using SPIN LTL syntax <http://spinroot.com/spin/Man/ltl.html>.
       Support for other syntax is coming soon. */
    void to_formula( std::ostream &out ) const;
};

Problem::Problem( RoadNetwork &rnd_ )
    : rnd(rnd_)
{
    goals.push_back( 0 );
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


} // namespace dubins_traffic


#endif
