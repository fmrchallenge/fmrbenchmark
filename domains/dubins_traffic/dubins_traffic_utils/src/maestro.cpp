/* maestro.cpp - management of multiple trials for dubins_traffic
 *
 * SCL; 2016
 */

#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "dubins_traffic_msgs/MMode.h"
#include "roadnet.hpp"
#include "problem.hpp"


using namespace dubins_traffic;


class Maestro {
public:
    Maestro( ros::NodeHandle &nh_, Problem &problem_ );

    bool mode_request( dubins_traffic_msgs::MMode::Request &req,
                       dubins_traffic_msgs::MMode::Response &res );

private:
    ros::NodeHandle &nh;
    Problem &problem;
    ros::ServiceServer mode_srv;
    ros::ServiceClient gazebo_toggle;
    ros::Publisher set_model_state;
    bool gazebo_paused;
};

Maestro::Maestro( ros::NodeHandle &nh_, Problem &problem_ )
    : nh(nh_), problem(problem_), gazebo_paused( true )
{
    mode_srv = nh.advertiseService( "mode", &Maestro::mode_request, this );
    set_model_state = nh.advertise<gazebo_msgs::ModelState>( "/gazebo/set_model_state", 1, true );
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

    std::string rndjson;
    while (!nh.getParam( "/dubins_traffic/rnd", rndjson )) {
        ROS_INFO( "dubins_traffic monitor: RND JSON string not present; will try again soon..." );
        sleep( 1 );
        if (!ros::ok())
            return 0;
    }
    RoadNetwork rnd( rndjson );
    Problem problem( rnd );

    Maestro maestro( nh, problem );
    ros::spin();

    return 0;
}
