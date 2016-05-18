/* maestro.cpp - management of multiple trials for dubins_traffic
 *
 * SCL; 2016
 */

#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <Eigen/Dense>

#include "integrator_chains_msgs/ProblemInstanceJSON.h"
#include "dubins_traffic_msgs/MMode.h"
#include "roadnet.hpp"
#include "problem.hpp"


using namespace dubins_traffic;


class Maestro {
public:
    Maestro( ros::NodeHandle &nh_, const RoadNetwork rnd_ );

    bool mode_request( dubins_traffic_msgs::MMode::Request &req,
                       dubins_traffic_msgs::MMode::Response &res );

    void perform_trial( std::ofstream &outf );
    void main();

private:
    ros::NodeHandle &nh;
    RoadNetwork rnd;
    Problem probinstance;
    ros::Publisher problemJSONpub;

    ros::Publisher set_model_state;
    ros::ServiceClient gazebo_toggle;
    bool gazebo_paused;

    ros::ServiceServer mode_srv;
    enum MMode { ready, waiting, running, resetting };
    MMode mmode;

    /* (copied from domains/integrator_chains/dynamaestro/src/main.cpp to avoid dependency.) */
    bool parse_range_str( const std::string param_name, Eigen::Vector2i &range );
};

bool Maestro::parse_range_str( const std::string param_name, Eigen::Vector2i &range )
{
    std::string range_str;
    if (!nh.getParam( param_name, range_str ))
        return false;

    char *endptr;
    errno = 0;
    range(0) = strtol( range_str.data(), &endptr, 10 );
    if (errno || endptr == range_str.data()) {
        std::cerr << "dubins_traffic_maestro: Malformed range string in \""
                  << param_name << "\"" << std::endl;
        return false;
    }
    char *prev_endptr = endptr;
    errno = 0;
    range(1) = strtol( prev_endptr, &endptr, 10 );
    if (errno || endptr == prev_endptr) {
        std::cerr << "dubins_traffic_maestro: Malformed range string in \""
                  << param_name << "\"" << std::endl;
        return false;
    }

    return true;
}

Maestro::Maestro( ros::NodeHandle &nh_, const RoadNetwork rnd_ )
    : nh(nh_), rnd(rnd_), probinstance( rnd_ ), gazebo_paused( true ), mmode(resetting)
{
    problemJSONpub = nh.advertise<integrator_chains_msgs::ProblemInstanceJSON>( "probleminstance_JSON", 1, true );
    mode_srv = nh.advertiseService( "mode", &Maestro::mode_request, this );
    set_model_state = nh.advertise<gazebo_msgs::ModelState>( "/gazebo/set_model_state", 1, false );
}

bool Maestro::mode_request( dubins_traffic_msgs::MMode::Request &req,
                            dubins_traffic_msgs::MMode::Response &res )
{
    switch (req.mode) {
    case dubins_traffic_msgs::MMode::Request::READY:
        if (mmode == ready) {
            res.result = true;
        } else {
            res.result = false;
        }
        break;

    case dubins_traffic_msgs::MMode::Request::START:
        if (gazebo_paused) {
            gazebo_toggle = nh.serviceClient<std_srvs::Empty>( "/gazebo/unpause_physics" );
            std_srvs::Empty empty;
            if (gazebo_toggle.call( empty ))
                gazebo_paused = false;
        }
        if (mmode == ready && !gazebo_paused) {
            mmode = waiting;
            res.result = true;
        } else {
            res.result = false;
        }
        break;

    case dubins_traffic_msgs::MMode::Request::RESET:
        if (!gazebo_paused) {
            gazebo_toggle = nh.serviceClient<std_srvs::Empty>( "/gazebo/pause_physics" );
            std_srvs::Empty empty;
            if (gazebo_toggle.call( empty ))
                gazebo_paused = true;
        }
        mmode = resetting;
        ROS_INFO( "dubins_traffic_maestro: Stopping current trial and generating a new instance..." );
        res.result = true;
        break;

    default:
        return false;
    }

    return true;
}

void Maestro::perform_trial( std::ofstream &outf )
{
    Eigen::Vector2i number_goals_bounds;
    if (!parse_range_str( "number_goals_bounds", number_goals_bounds )) {
        number_goals_bounds << 1, 4;
    }
    ROS_INFO( "dubins_traffic_maestro: Using [%d, %d] as sampling range "
              "for number of goal polytopes.",
              number_goals_bounds(0), number_goals_bounds(1) );

    Eigen::Vector2i duration_bounds;
    if (!parse_range_str( "duration_bounds", duration_bounds )) {
        duration_bounds << 30, 90;
    }
    ROS_INFO( "dubins_traffic_maestro: Using [%d, %d] as range of integers for sampling "
              "the trial duration.",
              duration_bounds(0), duration_bounds(1) );

    int nominal_duration = duration_bounds(0);
    if (duration_bounds(0) != duration_bounds(1))
        nominal_duration += std::rand() % (1+duration_bounds(1)-duration_bounds(0));

    probinstance = Problem::random( rnd, number_goals_bounds );

    ros::Duration trial_duration;

    mmode = ready;
    while (mmode == ready) {
        if (!ros::ok())
            return;
        ros::spinOnce();
    }
    assert( mmode == waiting );

    integrator_chains_msgs::ProblemInstanceJSON probinstance_msg;
    probinstance_msg.stamp = ros::Time::now();
    probinstance_msg.problemjson = probinstance.dumpJSON();
    problemJSONpub.publish( probinstance_msg );
    ros::Time startt( probinstance_msg.stamp.sec,
                      probinstance_msg.stamp.nsec );
    ros::spinOnce();

    outf << probinstance << std::endl;

    ros::Rate polling_rate( 1000 );
    while (ros::ok() && mmode != resetting && (trial_duration = ros::Time::now() - startt).toSec() < nominal_duration) {
        if (mmode == waiting) {
            mmode = running;
        }
        ros::spinOnce();
        polling_rate.sleep();
    }
    mmode = resetting;

    // Send empty instance description to indicate that trial has ended.
    probinstance_msg.stamp = ros::Time::now();
    probinstance_msg.problemjson = "";
    problemJSONpub.publish( probinstance_msg );

    gazebo_toggle = nh.serviceClient<std_srvs::Empty>( "/gazebo/pause_physics" );
    std_srvs::Empty empty;
    if (gazebo_toggle.call( empty ))
        gazebo_paused = true;

    // Handle special cases of trial termination:
    if (ros::ok() && trial_duration.toSec() >= nominal_duration) {
        // The trial end because the nominal (hidden) duration was reached
        ROS_INFO( "dubins_traffic_maestro: Trial ended after %f s (threshold is %d s).",
                  trial_duration.toSec(), nominal_duration );
    }
}

void Maestro::main()
{
    int number_trials = 0;
    bool counting_trials = nh.getParam( "/number_trials", number_trials );
    if (counting_trials)
        ROS_INFO( "dubins_traffic_maestro: Initiated with request for %d trials.",
                  number_trials );

    std::string filename;
    nh.getParam( "/dubins_traffic/results_file", filename );
    std::ofstream outf( filename.c_str(),
                        std::ios_base::out | std::ios_base::app );
    outf << "\"instances\": [";

    int trial_counter = 0;
    while ((!counting_trials || trial_counter < number_trials)
           && ros::ok()) {
        if (trial_counter > 0)
            outf << ", ";
        perform_trial( outf );
        if (counting_trials)
            trial_counter++;
    }

    outf << "]}";
    outf.close();

    if (counting_trials && ros::ok()) {
        ROS_INFO( "dubins_traffic_maestro: Completed %d trials.", trial_counter );
    }
}


int main( int argc, char **argv )
{
    ros::init( argc, argv, "dubins_traffic_maestro" );
    ros::NodeHandle nh( "~" );

    std::string rndjson;
    while (!nh.getParam( "/dubins_traffic/rnd", rndjson )) {
        ROS_INFO( "dubins_traffic_maestro: RND JSON string not present; will try again soon..." );
        sleep( 1 );
        if (!ros::ok())
            return 0;
    }
    RoadNetwork rnd( rndjson );
    Maestro maestro( nh, rnd );

    maestro.main();
    nh.shutdown();

    return 0;
}
