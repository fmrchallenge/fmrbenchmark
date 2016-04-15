#ifndef ROADNET_H
#define ROADNET_H

#include <string>
#include <vector>
#include <iostream>
#include <cassert>

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

    /** Create RoadNetwork from a description in a JSON container.

        segments is not yet implemented. Thus, until then, it is only possible
        to define a 4-connected grid using shape.
     */
    RoadNetwork( std::istream &rndjson );

    /** Output in RND format using JSON container to given stream. */
    friend std::ostream & operator<<( std::ostream &out, const RoadNetwork &rd );

private:
    void populate_4grid( int shape0, int shape1 );

private:
    int version;
    double length;
    Eigen::Vector3d transform;
    std::vector<Eigen::Vector4d> segments;
    std::vector<int> shape;
};

void RoadNetwork::populate_4grid( int shape0, int shape1 )
{
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

RoadNetwork::RoadNetwork( double length_,
                          const Eigen::Vector3d &transform_,
                          int shape0, int shape1 )
    : version(0), length(length_), transform(transform_),
      shape( {shape0, shape1} )
{
    assert( length > 0 );
    populate_4grid( shape0, shape1 );
}

RoadNetwork::RoadNetwork( std::istream &rndjson )
    : version(-1), length(1.0), transform(0.0, 0.0, 0.0),
      shape( {2, 2} )
{
    enum state {waiting, rd_version, rd_length, rd_transform, rd_segments, rd_shape};
    state parser = waiting;
    std::string buf;
    std::string rndjson_str;
    rndjson >> buf;
    while (!rndjson.eof()) {
        rndjson_str.append( buf );
        rndjson >> buf;
    }

    size_t pos = 0, nextpos;
    while (pos < rndjson_str.size()) {
        if (parser == waiting) {
            if ((nextpos = rndjson_str.find( "version", pos )) != std::string::npos) {
                parser = rd_version;
                pos = nextpos + 7;
            } else if ((nextpos = rndjson_str.find( "length", pos )) != std::string::npos) {
                parser = rd_length;
                pos = nextpos + 6;
            } else if ((nextpos = rndjson_str.find( "transform", pos )) != std::string::npos) {
                parser = rd_transform;
                pos = nextpos + 9;
            } else if ((nextpos = rndjson_str.find( "shape", pos )) != std::string::npos) {
                parser = rd_shape;
                pos = nextpos + 5;
            } else {
                break;
            }

        } else if (parser == rd_version || parser == rd_length) {
            if ((nextpos = rndjson_str.find( ":", pos )) != std::string::npos) {
                pos = nextpos + 1;
                try {
                    int parsed_int = std::stoi( rndjson_str.substr( pos ) );
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
                              << rndjson_str.substr( pos ) << "\"" << std::endl;
                }
                parser = waiting;
            } else {
                break;
            }

        } else if (parser == rd_transform || parser == rd_shape) {
            std::vector<double> parsed_numbers;
            if ((nextpos = rndjson_str.find( "[", pos )) != std::string::npos) {
                pos = nextpos + 1;
                while (true) {
                    try {
                        parsed_numbers.push_back( std::stod( rndjson_str.substr( pos ) ) );
                    } catch (std::invalid_argument) {
                        std::cout << "stod() conversion failed on \""
                                  << rndjson_str.substr( pos ) << "\"" << std::endl;
                    }

                    size_t closingpos = rndjson_str.find( "]", pos );
                    nextpos = rndjson_str.find( ",", pos );
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
