#ifndef ROADNET_H
#define ROADNET_H

#include <string>
#include <iostream>

#include <Eigen/Dense>


/** Road network description

   This class serves a similar purpose to the class RoadNetwork in
   dubins_traffic.py of the fmrb Python package. However, this is considered the
   reference implementation.

   \ingroup dubins_traffic
 */
class RoadNetwork
{
public:
    /** Create RoadNetwork that is 4-connected grid of size (shape0, shape1). */
    RoadNetwork( double length_,
                 const Eigen::Vector3d &transform_,
                 int shape0, int shape1 );

    /** Output in RND format using JSON container to given stream. */
    friend std::ostream & operator<<( std::ostream &out, const RoadNetwork &rd );

private:
    int version;
    double length;
    Eigen::Vector3d transform;
    std::vector<Eigen::Vector4d> segments;
    std::vector<int> shape;
};

RoadNetwork::RoadNetwork( double length_,
                          const Eigen::Vector3d &transform_,
                          int shape0, int shape1 )
    : version(0), length(length_), transform(transform_),
      shape( {shape0, shape1} )
{
    assert( length > 0 );
    assert( shape0 >= 1 && shape1 >= 1 );
    for (int x = 0; x < shape1-1; x++) {
        for (int y = 0; y < shape0-1; y++) {
            segments.push_back( Eigen::Vector4d( x, y, x+1, y ) );
            segments.push_back( Eigen::Vector4d( x, y, x, y+1 ) );
        }
    }
    for (int x = 0; x < shape1-1; x++)
        segments.push_back( Eigen::Vector4d( x, shape0-1, x+1, shape0-1 ) );
    for (int y = 0; y < shape0-1; y++)
        segments.push_back( Eigen::Vector4d( shape1-1, y, shape1-1, y+1 ) );
}

std::ostream & operator<<( std::ostream &out, const RoadNetwork &rd )
{
    out << "{ \"version\": " << rd.version << ", ";
    out << "\"length\": " << rd.length << ", ";
    out << "\"transform\": ["
        << rd.transform(0) << ", " << rd.transform(1) << ", " << rd.transform(2)
        << "], ";
    out << "\"shape\": [" << rd.shape[0] << ", " << rd.shape[1] << "], ";
    out << "\"segments\": [";
    for (int segment_index = 0; segment_index < rd.segments.size(); segment_index++) {
        out << "[";
        for (int idx = 0; idx < 4; idx++) {
            out << rd.segments[segment_index](idx);
            if (idx < 3)
                out << ", ";
        }
        out << "]";
        if (segment_index < rd.segments.size()-1)
            out << ", ";
    }
    out << "] }";
}


#endif
