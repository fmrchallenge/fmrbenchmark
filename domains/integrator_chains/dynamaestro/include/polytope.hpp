#ifndef POLYTOPE_H
#define POLYTOPE_H

#include <cassert>
#include <iostream>
#include <vector>

#include <Eigen/Dense>


/** Basic half-space representation of polytopes.

   \ingroup integrator_chains */
class Polytope {
  public:
    Polytope( Eigen::MatrixXd incoming_H, Eigen::VectorXd incoming_K );
    Polytope( const Polytope &other );

    /** Check that defining matrices have consistent dimensions.

       The "defining matrices" are those involved in the inequality providing
       the half-space representation, i.e., H and K in {x | Hx <= K}.
       N.B., this check does not occur during instantiation of Polytope. */
    bool is_consistent() const;

    /** Is X contained in this polytope?

       The polytope is a closed set. No numerical tolerance is used. */
    bool is_in( Eigen::VectorXd X ) const;

    // Factory functions
    /** Create random Polytope in R^n using Eigen::MatrixXd::Random().

       \param n dimension of the containing space.

       Random matrices as provided by Eigen::MatrixXd::Random() are used to
       instantiate Polytope in half-space representation. The matrices have
       \c n+1 rows, corresponding to \c n+1 inequalities.
    */
    static Polytope * randomH( int n );

    /** Construct Polytope that is an axis-aligned rectangle.

       \param bounds an array of the form
       \code
       [x1_min, x1_max, x2_min, x2_max, ..., xn_min, xn_max],
       \endcode
       which defines a polytope in terms of intervals along each axis in R^n.
       The interval along the first axis is [x1_min, x1_max], the interval along
       the second axis is [x2_min, x2_max], etc. Thus the length of the given
       array is 2n.

       E.g., a unit square in R^2 can be created using
       \code
           Eigen::Vector4d bounds;
           bounds << 0, 1,
                     0, 1;
           Polytope *square = Polytope::box( bounds );
       \endcode
    */
    static Polytope * box( const Eigen::VectorXd &bounds );

    /** Output this polytope in JSON to given stream. */
    friend std::ostream & operator<<( std::ostream &out, const Polytope &P );

    /** Output this polytope in JSON excluding opening { and closing }.

       This method facilitates inheritance from Polytope without crowding the
       JSON representations of other classes. */
    void dumpJSONcore( std::ostream &out ) const;

  private:
    // { x \in R^n | Hx \leq K }
    Eigen::MatrixXd H;
    Eigen::VectorXd K;
};

std::ostream & operator<<( std::ostream &out, const Polytope &P )
{
    out << "{ ";
    P.dumpJSONcore( out );
    out << " }";
    return out;
}

void Polytope::dumpJSONcore( std::ostream &out ) const
{
    int i, j;
    out << "\"H\": [";
    for (i = 0; i < H.rows(); i++) {
        if (i > 0) {
            out << ", ";
        }
        out << "[";
        for (j = 0; j < H.cols(); j++) {
            if (j > 0)
                out << ", ";
            out << H(i,j);
        }
        out << "]";
    }
    out << "], \"K\": [";
    for (i = 0; i < K.rows(); i++) {
        if (i > 0) {
            out << ", ";
        }
        out << K(i);
    }
    out << "]";
}

Polytope * Polytope::randomH( int n )
{
    return new Polytope( Eigen::MatrixXd::Random( n+1, n ),
                         Eigen::VectorXd::Random( n+1 ) );
}

Polytope * Polytope::box( const Eigen::VectorXd &bounds )
{
    assert( bounds.size() % 2 == 0 );

    int n = bounds.size()/2;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero( 2*n, n );
    Eigen::VectorXd K( 2*n );
    for (int i = 0; i < n; i++) {
        H(2*i, i) = -1;
        K(2*i) = -bounds(2*i);
        H(2*i+1, i) = 1;
        K(2*i+1) = bounds(2*i+1);
    }
    return new Polytope( H, K );
}

bool Polytope::is_consistent() const
{
    if (H.rows() == K.size()) {
        return true;
    } else {
        return false;
    }
}

bool Polytope::is_in( Eigen::VectorXd X ) const
{
    assert( H.cols() == X.size() );

    if ((H*X-K).maxCoeff() <= 0) {
        return true;
    } else {
        return false;
    }
}

Polytope::Polytope( Eigen::MatrixXd incoming_H, Eigen::VectorXd incoming_K )
    : H(incoming_H), K(incoming_K)
{ }

Polytope::Polytope( const Polytope &other )
    : H(other.H), K(other.K)
{ }


/** Extension of Polytope to have label (string).

   This class provides a basis for labeling trajectories. E.g., the labeling of
   the state can be a set of strings corresponding to the labels of polytopes
   containing the state.

   \ingroup integrator_chains */
class LabeledPolytope : public Polytope {
public:
    std::string label;

    LabeledPolytope( const Polytope &other );

    /** Polytope::box() but including a label. */
    static LabeledPolytope * box( const Eigen::VectorXd &bounds, std::string label="" );

    /** Output this labeled polytope in JSON to given stream. */
    friend std::ostream & operator<<( std::ostream &out, const LabeledPolytope &P );
};

LabeledPolytope::LabeledPolytope( const Polytope &other )
    : label(""), Polytope( other )
{ }

LabeledPolytope * LabeledPolytope::box( const Eigen::VectorXd &bounds, std::string label )
{
    Polytope *P = Polytope::box( bounds );
    LabeledPolytope *Q = new LabeledPolytope( *P );
    delete P;
    Q->label = label;
    return Q;
}

std::ostream & operator<<( std::ostream &out, const LabeledPolytope &P )
{
    out << "{ \"label\": \"" << P.label << "\", ";
    P.dumpJSONcore( out );
    out << " }";
    return out;
}


#endif  // ifndef POLYTOPE_H
