/* monitor.cpp - main routine for problem instance representation and monitoring
 *
 * SCL; 2016
 */

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

#include "roadnet.hpp"


int main( int argc, char **argv )
{
    ros::init( argc, argv, "dubins_traffic_monitor" );
    ros::NodeHandle nh( "~" );

    std::string rndjson;
    while (!nh.getParam( "/dubins_traffic/rnd", rndjson )) {
        ROS_INFO( "dubins_traffic monitor: RND JSON string not present; will try again soon..." );
        sleep( 1 );
        if (!ros::ok())
            return 0;
    }
    RoadNetwork rnd( rndjson );

    std::string eagents_str;
    while (!nh.getParam( "/dubins_traffic/e_agents", eagents_str )) {
        ROS_INFO( "dubins_traffic monitor: e-agents list not present; will try again soon..." );
        sleep( 1 );
        if (!ros::ok())
            return 0;
    }
    std::vector<std::string> eagent_names;
    if (eagents_str.size() > 0) {
        size_t pos = 0, nextpos;
        while (true) {
            nextpos = eagents_str.find( ",", pos );
            if (nextpos == std::string::npos) {
                eagent_names.push_back( eagents_str.substr( pos ) );
                break;
            }
            eagent_names.push_back( eagents_str.substr( pos, nextpos-pos ) );
            pos = nextpos + 1;
        }

    }

    return 0;
}
