/* maestro.cpp - management of multiple trials for dubins_traffic
 *
 * SCL; 2016
 */

#include <ros/ros.h>
#include "std_srvs/Empty.h"

#include "dubins_traffic_msgs/MMode.h"


class Maestro {
public:
    Maestro( ros::NodeHandle &nh_ );

    bool mode_request( dubins_traffic_msgs::MMode::Request &req,
                       dubins_traffic_msgs::MMode::Response &res );

private:
    ros::NodeHandle &nh;
    ros::ServiceServer mode_srv;
    ros::ServiceClient gazebo_toggle;
    bool gazebo_paused;
};

Maestro::Maestro( ros::NodeHandle &nh_ )
    : nh(nh_), gazebo_paused( true )
{
    mode_srv = nh.advertiseService( "mode", &Maestro::mode_request, this );
}

bool Maestro::mode_request( dubins_traffic_msgs::MMode::Request &req,
                            dubins_traffic_msgs::MMode::Response &res )
{
    if (gazebo_paused) {
        gazebo_toggle = nh.serviceClient<std_srvs::Empty>( "/gazebo/unpause_physics" );
    } else {
        gazebo_toggle = nh.serviceClient<std_srvs::Empty>( "/gazebo/pause_physics" );
    }
    std_srvs::Empty empty;
    if (gazebo_toggle.call( empty ))
        gazebo_paused = !gazebo_paused;
    res.result = true;
    return true;
}


int main( int argc, char **argv )
{
    ros::init( argc, argv, "dubins_traffic_maestro" );
    ros::NodeHandle nh( "~" );

    Maestro maestro( nh );
    ros::spin();

    return 0;
}
