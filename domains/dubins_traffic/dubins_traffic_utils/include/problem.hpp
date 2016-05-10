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

    /** Output description of problem in JSON to a stream */
    friend std::ostream & operator<<( std::ostream &out, const Problem &prob );

    /** Get description of problem in JSON. */
    std::string dumpJSON() const;
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
