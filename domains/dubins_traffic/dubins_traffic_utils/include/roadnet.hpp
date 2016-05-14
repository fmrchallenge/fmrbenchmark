#ifndef ROADNET_H
#define ROADNET_H

#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <cassert>

#include <Eigen/Dense>


namespace dubins_traffic {


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

    /** Create RoadNetwork from a description in a JSON container.

        segments is not yet implemented. Thus, until then, it is only possible
        to define a 4-connected grid using shape.
     */
    RoadNetwork( std::istream &rndjson );
    RoadNetwork( const std::string &rndjson );

    RoadNetwork( const RoadNetwork &to_be_copied );

    /** Output in RND format using JSON container to given stream. */
    friend std::ostream & operator<<( std::ostream &out, const RoadNetwork &rd );

    int number_of_segments() const
        { return segments.size(); }

    int number_of_intersections() const
        { return intersections.size(); }

    /** Get mapped road segment. */
    const Eigen::Vector4d mapped_segment( size_t idx, int lane = 0 ) const;

    std::vector<size_t> segments_at_start( size_t idx ) const;
    std::vector<size_t> segments_at_end( size_t idx, bool reverse = false ) const;

    /** Map point in local coordinates through transform and scaling.

       This function does not check whether the given point is on some segment.
     */
    void map_point( const int x, const int y,
                    double &mapped_x, double &mapped_y ) const;

    /** Get index of segment nearest to x,y point. */
    size_t get_nearest_segment( double x, double y ) const;

    /** Get minimum distance from point to center of road segment. */
    double get_segment_mindist( size_t idx, double x, double y ) const;

    /** Get index of intersection nearest to x,y point. */
    size_t get_nearest_intersection( double x, double y ) const;

    /** Get minimum distance from point to center of road intersection. */
    double get_intersection_mindist( size_t idx, double x, double y ) const;

    std::string get_segment_str( size_t idx ) const;
    std::string get_intersection_str( size_t idx ) const;

private:
    void populate_4grid( int shape0, int shape1 );
    void parse_json( const std::string &rndjson );

private:
    int version;
    double length;
    Eigen::Vector3d transform;
    std::vector<Eigen::Vector4i> segments;
    std::vector<Eigen::Vector2i> intersections;
    std::vector<int> shape;
};

RoadNetwork::RoadNetwork( const RoadNetwork &to_be_copied )
{
    version = to_be_copied.version;
    length = to_be_copied.length;
    transform = to_be_copied.transform;
    segments = to_be_copied.segments;
    intersections = to_be_copied.intersections;
    shape = to_be_copied.shape;
}

std::vector<size_t> RoadNetwork::segments_at_end( size_t idx, bool reverse ) const
{
    assert( idx >= 0 && idx < number_of_segments() );
    int offset = 0;
    if (reverse)
        offset = -2;
    std::vector<size_t> adjacent_indices;
    for (size_t jj = 0; jj < number_of_segments(); jj++) {
        if (jj == idx)
            continue;
        if (segments[idx].segment<2>(2+offset) == segments[jj].segment<2>(0)
            || segments[idx].segment<2>(2+offset) == segments[jj].segment<2>(2))
            adjacent_indices.push_back( jj );
    }
    return adjacent_indices;
}

std::vector<size_t> RoadNetwork::segments_at_start( size_t idx ) const
{
    return segments_at_end( idx, true );
}

const Eigen::Vector4d RoadNetwork::mapped_segment( size_t idx, int lane ) const
{
    Eigen::Vector4d mapped;
    for (size_t offset = 0; offset < 2; offset++)
        map_point( segments[idx](2*offset), segments[idx](2*offset+1),
                   mapped(2*offset), mapped(2*offset+1) );

    if (lane != 0) {
        const double lane_width = 0.6;

        Eigen::Vector2d orthog( mapped(1) - mapped(3), mapped(2) - mapped(0) );
        Eigen::Vector2d offset = orthog/orthog.norm()*lane_width;
        if (lane > 0) {
            offset *= lane-0.3;
        } else {  // lane < 0
            offset *= lane+0.3;
        }
        for (size_t idx = 0; idx < 4; idx++)
            mapped(idx) += offset(idx%2);

    }
    return mapped;
}

void RoadNetwork::map_point( const int x, const int y,
                             double &mapped_x, double &mapped_y ) const
{
    double costheta = std::cos( transform(2) );
    double sintheta = std::sin( transform(2) );
    mapped_x = length*x;
    mapped_y = length*y;
    double new_x = costheta*mapped_x - sintheta*mapped_y;
    double new_y = sintheta*mapped_x + costheta*mapped_y;
    mapped_x = new_x + transform(0);
    mapped_y = new_y + transform(1);
}

void RoadNetwork::populate_4grid( int shape0, int shape1 )
{
    assert( shape0 >= 1 && shape1 >= 1 );
    for (int x = 0; x < shape1-1; x++) {
        for (int y = 0; y < shape0-1; y++) {
            segments.push_back( Eigen::Vector4i( x, y, x+1, y ) );
            segments.push_back( Eigen::Vector4i( x, y, x, y+1 ) );
        }
    }
    for (int x = 0; x < shape1-1; x++)
        segments.push_back( Eigen::Vector4i( x, shape0-1, x+1, shape0-1 ) );
    for (int y = 0; y < shape0-1; y++)
        segments.push_back( Eigen::Vector4i( shape1-1, y, shape1-1, y+1 ) );

    for (int x = 0; x < shape1; x++)
        for (int y = 0; y < shape0; y++)
            intersections.push_back( Eigen::Vector2i( x, y ) );
}

RoadNetwork::RoadNetwork( double length_,
                          const Eigen::Vector3d &transform_,
                          int shape0, int shape1 )
    : version(0), length(length_), transform(transform_),
      shape( {shape0, shape1} )
{
    assert( length > 0 );
    populate_4grid( shape0, shape1 );
}

RoadNetwork::RoadNetwork( const std::string &rndjson )
    : version(-1), length(1.0), transform(0.0, 0.0, 0.0),
      shape( {2, 2} )
{
    parse_json( rndjson );
}

RoadNetwork::RoadNetwork( std::istream &rndjson )
    : version(-1), length(1.0), transform(0.0, 0.0, 0.0),
      shape( {2, 2} )
{
    std::string buf;
    std::string rndjson_str;
    do {
        rndjson >> buf;
        rndjson_str.append( buf );
    } while (!rndjson.eof());

    parse_json( rndjson_str );
}

void RoadNetwork::parse_json( const std::string &rndjson )
{
    enum state {waiting, rd_version, rd_length, rd_transform, rd_segments, rd_shape};
    state parser = waiting;
    bool found_version = false;
    bool found_length = false, found_transform = false, found_shape = false;

    size_t pos = 0, nextpos;
    while (true) {
        if (parser == waiting) {
            pos = 0;
            if (!found_version
                && (nextpos = rndjson.find( "version", pos )) != std::string::npos) {
                found_version = true;
                parser = rd_version;
                pos = nextpos + 7;
            } else if (!found_length
                       && (nextpos = rndjson.find( "length", pos )) != std::string::npos) {
                found_length = true;
                parser = rd_length;
                pos = nextpos + 6;
            } else if (!found_transform
                       && (nextpos = rndjson.find( "transform", pos )) != std::string::npos) {
                found_transform = true;
                parser = rd_transform;
                pos = nextpos + 9;
            } else if (!found_shape
                       && (nextpos = rndjson.find( "shape", pos )) != std::string::npos) {
                found_shape = true;
                parser = rd_shape;
                pos = nextpos + 5;
            } else {
                break;
            }

        } else if (parser == rd_version || parser == rd_length) {
            if ((nextpos = rndjson.find( ":", pos )) != std::string::npos) {
                pos = nextpos + 1;
                try {
                    int parsed_int = std::stoi( rndjson.substr( pos ) );
                    switch (parser) {
                    case rd_version:
                        version = parsed_int;
                        break;
                    case rd_length:
                        length = parsed_int;
                        break;
                    }
                } catch (std::invalid_argument) {
                    std::cout << "stoi() conversion failed on \""
                              << rndjson.substr( pos ) << "\"" << std::endl;
                }
                parser = waiting;
            } else {
                break;
            }

        } else if (parser == rd_transform || parser == rd_shape) {
            std::vector<double> parsed_numbers;
            if ((nextpos = rndjson.find( "[", pos )) != std::string::npos) {
                pos = nextpos + 1;
                while (true) {
                    try {
                        parsed_numbers.push_back( std::stod( rndjson.substr( pos ) ) );
                    } catch (std::invalid_argument) {
                        std::cout << "stod() conversion failed on \""
                                  << rndjson.substr( pos ) << "\"" << std::endl;
                    }

                    size_t closingpos = rndjson.find( "]", pos );
                    nextpos = rndjson.find( ",", pos );
                    if (closingpos == std::string::npos) {
                        break;  // Missing ] implies malformed array.
                    } else if (nextpos != std::string::npos && nextpos < closingpos) {
                        pos = nextpos + 1;
                    } else {
                        pos = closingpos + 1;
                        break;  // Found closing ] implies well-formed array.
                    }
                }

                switch (parser) {
                case rd_transform:
                    assert( parsed_numbers.size() == 3 );
                    for (int idx = 0; idx < parsed_numbers.size(); idx++)
                        transform(idx) = parsed_numbers.at(idx);
                    break;
                case rd_shape:
                    assert( parsed_numbers.size() == 2 );
                    for (int idx = 0; idx < parsed_numbers.size(); idx++)
                        shape[idx] = parsed_numbers.at(idx);
                    break;
                }

                parser = waiting;
            } else {
                break;
            }

        } else {
            break;
        }
    }

    assert( version == 0 );
    assert( length > 0 );
    populate_4grid( shape[0], shape[1] );
}

size_t RoadNetwork::get_nearest_segment( double x, double y ) const
{
    size_t min_idx;
    double min_dist = -1.0;
    for (size_t idx = 0; idx < number_of_segments(); idx++) {
        double this_dist = get_segment_mindist( idx, x, y );
        if (min_dist < 0 || this_dist < min_dist) {
            min_idx = idx;
            min_dist = this_dist;
        }
    }
    return min_idx;
}

double RoadNetwork::get_segment_mindist( size_t idx, double x, double y ) const
{
    assert( idx >= 0 && idx < number_of_segments() );
    Eigen::Vector4d road = mapped_segment( idx );
    Eigen::Vector2d pt( x, y );
    return ((road.segment<2>(0) + road.segment<2>(2))/2.0 - pt).norm();
    // road(2) -= road(0);
    // road(3) -= road(1);
    // pt -= road.segment<2>(0);
    // if (road.segment<2>(2).dot( pt ) < 0) {
    //     return pt.norm();
    // } else if (pt.dot( road.segment<2>(2) - pt ) < 0) {
    //     return (road.segment<2>(2) - pt).norm();
    // } else {
    //     Eigen::Vector2d unitv = road.segment<2>(2)/road.segment<2>(2).norm();
    //     return (pt - pt.dot( unitv )*unitv).norm();
    // }
}

std::string RoadNetwork::get_segment_str( size_t idx ) const
{
    assert( idx >= 0 && idx < number_of_segments() );
    std::ostringstream out;
    out << "s_";
    out << segments[idx](0) << "_" << segments[idx](1);
    out << "_" << segments[idx](2) << "_" << segments[idx](3);
    return out.str();
}

size_t RoadNetwork::get_nearest_intersection( double x, double y ) const
{
    size_t min_idx;
    double min_dist = -1.0;
    for (size_t idx = 0; idx < number_of_intersections(); idx++) {
        double this_dist = get_intersection_mindist( idx, x, y );
        if (min_dist < 0 || this_dist < min_dist) {
            min_idx = idx;
            min_dist = this_dist;
        }
    }
    return min_idx;
}

double RoadNetwork::get_intersection_mindist( size_t idx, double x, double y ) const
{
    assert( idx >= 0 && idx < number_of_intersections() );
    double mapped_x, mapped_y;
    map_point( intersections[idx](0), intersections[idx](1),
               mapped_x, mapped_y );
    return std::sqrt( (mapped_x - x)*(mapped_x - x) + (mapped_y - y)*(mapped_y - y) );
}

std::string RoadNetwork::get_intersection_str( size_t idx ) const
{
    assert( idx >= 0 && idx < number_of_intersections() );
    std::ostringstream out;
    out << "t_";
    out << intersections[idx](0) << "_" << intersections[idx](1);
    return out.str();
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


} // namespace dubins_traffic


#endif
