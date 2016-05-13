/* e-agent that moves along road segments and randomly turns at intersections.
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <string>
#include <cmath>
#include <cassert>
#include <unistd.h>

#include "roadnet.hpp"

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace dubins_traffic;


const double min_dist = 0.5;
bool safe_to_move = true;


void laser_cb( const sensor_msgs::LaserScan &scan )
{
    for (int i = 0; i < scan.ranges.size(); i++) {
        if (scan.ranges[i] < min_dist) {
            safe_to_move = false;
            return;
        }
    }
    safe_to_move = true;
}

class ModelStatesWatcher {
public:

    ModelStatesWatcher( ros::NodeHandle &nh, std::string topic_name = "/gazebo/model_states" );

    // Callback function for messages from the "/gazebo/model_states" topic
    void ms_cb( const gazebo_msgs::ModelStates &ms );

    /* Get current pose estimate
     *
     * If an estimate is available, return true and place the estimate in the
     * given array. Otherwise, return false and do nothing to the array.
     */
    bool get_pose_estimate( double *x );

    bool is_near_ego_vehicle() const
        { return near_ego_vehicle; }

private:
    double last_mspose[3];
    bool valid_mspose;
    std::string model_name;
    ros::Subscriber mssub;
    bool near_ego_vehicle;
};


ModelStatesWatcher::ModelStatesWatcher( ros::NodeHandle &nh, std::string topic_name )
    : valid_mspose(false), model_name(nh.getNamespace()), near_ego_vehicle(true),
      mssub(nh.subscribe( topic_name, 1, &ModelStatesWatcher::ms_cb, this ))
{
    while (model_name[0] == '/')
        model_name = std::string(model_name.c_str()+1);
}

void ModelStatesWatcher::ms_cb( const gazebo_msgs::ModelStates &ms )
{
    int this_model_idx = -1;
    int ego_model_idx = -1;
    for (int idx = 0; idx < ms.name.size(); idx++) {
        if (ms.name[idx] == model_name) {
            this_model_idx = idx;
            continue;
        }
        if (ms.name[idx] == "ego") {
            ego_model_idx = idx;
            continue;
        }
    }
    if (this_model_idx < 0)
        return;

    if (ego_model_idx >= 0
        && std::sqrt( (ms.pose[this_model_idx].position.x - ms.pose[ego_model_idx].position.x)*(ms.pose[this_model_idx].position.x - ms.pose[ego_model_idx].position.x)
                      + (ms.pose[this_model_idx].position.y - ms.pose[ego_model_idx].position.y)*(ms.pose[this_model_idx].position.y - ms.pose[ego_model_idx].position.y) ) > 1.0) {

        near_ego_vehicle = false;

    } else {

        near_ego_vehicle = true;  // Be cautious

    }

    valid_mspose = false;
    last_mspose[0] = ms.pose[this_model_idx].position.x;
    last_mspose[1] = ms.pose[this_model_idx].position.y;
    last_mspose[2] = tf::getYaw( ms.pose[this_model_idx].orientation );
    valid_mspose = true;
}

bool ModelStatesWatcher::get_pose_estimate( double *x )
{
    if (valid_mspose) {
        for (int i = 0; i < 3; i++)
            x[i] = last_mspose[i];
        return true;
    } else {
        return false;
    }
}


class OdomWatcher {
public:

    OdomWatcher( ros::NodeHandle &nh, std::string topic_name = "odom" );

    // Callback function for messages from the "odom" topic
    void odom_cb( const nav_msgs::Odometry &od );

    /* Get current pose estimate
     *
     * If an estimate is available, return true and place the estimate in the
     * given array. Otherwise, return false and do nothing to the array.
     */
    bool get_pose_estimate( double *x );

private:
    double last_opose[3];
    bool valid_opose;
    ros::Subscriber odomsub;
};

OdomWatcher::OdomWatcher( ros::NodeHandle &nh, std::string topic_name )
    : valid_opose(false),
      odomsub(nh.subscribe( topic_name, 1, &OdomWatcher::odom_cb, this ))
{ }

void OdomWatcher::odom_cb( const nav_msgs::Odometry &od )
{
    valid_opose = false;
    last_opose[0] = od.pose.pose.position.x;
    last_opose[1] = od.pose.pose.position.y;
    last_opose[2] = tf::getYaw( od.pose.pose.orientation );
    valid_opose = true;
}

bool OdomWatcher::get_pose_estimate( double *x )
{
    if (valid_opose) {
        for (int i = 0; i < 3; i++)
            x[i] = last_opose[i];
    }
    return valid_opose;
}


int main( int argc, char **argv )
{
    std::srand( getpid() );

    if (argc <= 1) {
        std::cerr << "Usage: " << argv[0] << " FILE" << std::endl;
        return 1;
    }

    std::ifstream infile( argv[1] );
    if (!infile.is_open()) {
        std::cerr << "Failed to open " << argv[1] << std::endl;
        return 1;
    }
    RoadNetwork rd( infile );
    assert( rd.number_of_segments() > 0 );

    std::pair<size_t, bool> current_segment_idx( std::rand() % rd.number_of_segments(), true );
    Eigen::Vector4d road_segment = rd.mapped_segment( current_segment_idx.first, current_segment_idx.second ? -1 : 1 );
    std::vector<Eigen::Vector2d> waypoints = { Eigen::Vector2d( road_segment(0), road_segment(1) ),
                                               Eigen::Vector2d( road_segment(2), road_segment(3) ) };
/* Pre-generated list of waypoints. This routine will be provided later as an alternative mode.
    std::vector< std::pair<size_t, bool> > visited_indices = {std::pair<size_t, bool>( 0, true )};
    Eigen::Vector4d road_segment = rd.mapped_segment( visited_indices.back().first );

    std::vector<Eigen::Vector2d> waypoints = { Eigen::Vector2d( road_segment(0), road_segment(1) ),
                                               Eigen::Vector2d( road_segment(2), road_segment(3) ) };

    while (true) {
        std::vector<size_t> end_indices;
        if (visited_indices.back().second) {  // Direction on previous segment?
            end_indices = rd.segments_at_end( visited_indices.back().first );
        } else {
            end_indices = rd.segments_at_start( visited_indices.back().first );
        }
        if (end_indices.size() == 0)  // Dead-end
            break;
        std::vector<size_t>::iterator end_it;
        for (end_it = end_indices.begin(); end_it != end_indices.end(); end_it++) {
            std::vector< std::pair<size_t, bool> >::iterator visited_it;
            for (visited_it = visited_indices.begin();
                 visited_it != visited_indices.end(); visited_it++) {
                if ((*visited_it).first == *end_it)
                    break;
            }
            if (visited_it == visited_indices.end())
                break;
        }
        if (end_it != end_indices.end()) {
            Eigen::Vector4d this_segment = rd.mapped_segment( *end_it );
            if ((this_segment.segment<2>(0) - waypoints.back()).norm() < 1e-6) {
                waypoints.push_back( this_segment.segment<2>(2) );
                visited_indices.push_back( std::pair<size_t, bool>( *end_it, true ) );
            } else {
                assert( (this_segment.segment<2>(2) - waypoints.back()).norm() < 1e-6 );
                waypoints.push_back( this_segment.segment<2>(0) );
                visited_indices.push_back( std::pair<size_t, bool>( *end_it, false ) );
            }
        } else {
            break;
        }
    }
*/

    // Configure turning rate and other motion parameters here
    double turning_rate = 0.9;  // rad/s
    double forward_speed = 0.5;  // m/s

    // Thresholds for switching among modes of motion
    double min_angle_err = 0.07;  // Minimum relative angle to drive forward toward waypoint
    double min_reach_err = 0.075;  // Distance from waypoint before it is declared as "reached"

    safe_to_move = true;
    ros::init( argc, argv, "Oscar", ros::init_options::AnonymousName );
    ros::NodeHandle nh;

    ros::Publisher action = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
    ros::Subscriber lmssub = nh.subscribe( "scan", 10, &laser_cb );

    ModelStatesWatcher pose_watcher( nh );

    int current_index = 0;  // Visit waypoints in order
    double current_pose[3];
    double angle_diff;

    ros::Rate rate( 30. );

    while (ros::ok()) {

        geometry_msgs::Twist mot;

        if (safe_to_move && pose_watcher.get_pose_estimate( current_pose )) {

            angle_diff = atan2( waypoints[current_index](1) - current_pose[1],
                                waypoints[current_index](0) - current_pose[0] ) - current_pose[2];
            while (angle_diff >= M_PI)
                angle_diff -= 2*M_PI;
            while (angle_diff < -M_PI)
                angle_diff += 2*M_PI;

            if (std::sqrt( (waypoints[current_index](0) - current_pose[0])*(waypoints[current_index](0) - current_pose[0])
                           + (waypoints[current_index](1) - current_pose[1])*(waypoints[current_index](1) - current_pose[1]) ) < min_reach_err) {
                std::cout << "Reached waypoint " << waypoints[current_index].transpose() << std::endl;
                // current_index = (current_index+1) % waypoints.size();
                if (current_index < 1) {
                    current_index++;
                } else {
                    std::vector<size_t> end_indices;
                    if (current_segment_idx.second) {  // Direction on previous segment?
                        end_indices = rd.segments_at_end( current_segment_idx.first );
                    } else {
                        end_indices = rd.segments_at_start( current_segment_idx.first );
                    }
                    if (end_indices.size() == 0) { // Dead-end
                        Eigen::Vector2d tmp = waypoints[0];
                        waypoints[0] = waypoints[current_index];
                        waypoints[current_index] = tmp;
                        current_segment_idx.second = current_segment_idx.second ? false : true;
                    } else {

                        Eigen::Vector4d this_segment = rd.mapped_segment( current_segment_idx.first );
                        waypoints[current_index] = this_segment.segment<2>( current_segment_idx.second ? 2 : 0 );
                        current_segment_idx.first = end_indices[std::rand() % end_indices.size()];
                        this_segment = rd.mapped_segment( current_segment_idx.first );
                        if ((this_segment.segment<2>(0) - waypoints[current_index]).norm() < 1e-6) {
                            current_segment_idx.second = true;
                        } else {
                            assert( (this_segment.segment<2>(2) - waypoints[current_index]).norm() < 1e-6 );
                            current_segment_idx.second = false;
                        }
                        this_segment = rd.mapped_segment( current_segment_idx.first, current_segment_idx.second ? -1 : 1 );
                        waypoints[current_index] = this_segment.segment<2>( current_segment_idx.second ? 2 : 0 );

                    }
                }
            } else if (fabs( angle_diff ) > min_angle_err) {

                if (angle_diff > 0) {
                    mot.angular.z = turning_rate;
                } else {
                    mot.angular.z = -turning_rate;
                }

            } else {
                mot.linear.x = forward_speed;
            }

        }

        if (pose_watcher.is_near_ego_vehicle())
            mot.linear.x = 0.0;

        action.publish( mot );
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
