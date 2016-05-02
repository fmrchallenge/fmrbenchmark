/* monitor.cpp - main routine for problem instance representation and monitoring
 *
 * SCL; 2016
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>

#include "dubins_traffic_msgs/LabelStamped.h"
#include "roadnet.hpp"


class Labeler {
public:
    Labeler( ros::NodeHandle &nh_,
             RoadNetwork &rnd_, const std::vector<std::string> &eagent_names_ );

private:
    std::vector<std::string> eagent_names;
    RoadNetwork &rnd;
    ros::NodeHandle &nh;
    ros::Subscriber subModelStates;
    ros::Publisher pubLabeledOutput;
    void labelcb( const gazebo_msgs::ModelStates &models );
};

Labeler::Labeler( ros::NodeHandle &nh_,
                  RoadNetwork &rnd_, const std::vector<std::string> &eagent_names_ )
    : nh(nh_), rnd(rnd_), eagent_names( eagent_names_ )
{
    subModelStates = nh.subscribe( "/gazebo/model_states", 1, &Labeler::labelcb, this );
    pubLabeledOutput = nh.advertise<dubins_traffic_msgs::LabelStamped>( "loutput", 1, true );
}

void Labeler::labelcb( const gazebo_msgs::ModelStates &models )
{
    std::vector<std::string> label;

    for (size_t idx = 0; idx < models.name.size(); idx++) {
        for (size_t agent_idx = 0; agent_idx < eagent_names.size(); agent_idx++) {
            if (models.name[idx] == eagent_names[agent_idx]) {

                size_t nearest = rnd.get_nearest_segment( models.pose[idx].position.x, models.pose[idx].position.y );
                {
                std::ostringstream out;
                out << eagent_names[agent_idx]
                    << "_"
                    << rnd.get_segment_str( nearest );
                label.push_back( out.str() );
                }

                nearest = rnd.get_nearest_intersection( models.pose[idx].position.x, models.pose[idx].position.y );
                {
                std::ostringstream out;
                out << eagent_names[agent_idx]
                    << "_"
                    << rnd.get_intersection_str( nearest );
                label.push_back( out.str() );
                }

                break;
            }
        }
        if (models.name[idx] == "ego") {
            size_t nearest = rnd.get_nearest_segment( models.pose[idx].position.x, models.pose[idx].position.y );
            {
            std::ostringstream out;
            out << "ego_"
                << rnd.get_segment_str( nearest );
            label.push_back( out.str() );
            }

            nearest = rnd.get_nearest_intersection( models.pose[idx].position.x, models.pose[idx].position.y );
            {
            std::ostringstream out;
            out << "ego_"
                << rnd.get_intersection_str( nearest );
            label.push_back( out.str() );
            }
        }
    }

    dubins_traffic_msgs::LabelStamped msg;
    msg.header.frame_id = std::string( "map" );
    msg.header.stamp = ros::Time::now();
    msg.label = label;
    pubLabeledOutput.publish( msg );
}


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

    Labeler labeler( nh, rnd, eagent_names );
    ros::spin();

    return 0;
}
