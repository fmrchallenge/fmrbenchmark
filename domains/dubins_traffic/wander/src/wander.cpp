/* Program that generates a series of waypoints on a grid relative to the initial position,
 * and  issues velocity commands for following this series of waypoints
 *
 * SCL; 6 Mar 2015
 * VR; 3 May, 26 May 2015 
*/

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


double min_dist = 0.5;
bool safe_to_move;


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



class PositionR2 {
public:
	PositionR2( double x0, double x1 )
	{
		x[0] = x0;
		x[1] = x1;
	}

	double x[2];
};


int main( int argc, char **argv )
{
	/*************************************************************/
	unsigned int seed;

	if (argc <= 1) {
          fprintf(stderr, "Usage: %s <seed> \n", argv[0]);
          exit(EXIT_FAILURE);
        }

        seed = atoi(argv[1]);
	srand(seed);
	
	// Generate a random series of waypoints on a grid.
	std::vector<PositionR2 *> waypoints;
	int numWaypts = 3;
	int x,y;
	int r1,r2;

	// Assumes initial position is (0,0) on the grid.
	x = 0;
	y = 0;
	
	int cell_size = 3; //size of grid cell in meters, assuming square cells
	
	for (int i=0; i<numWaypts; i++) {
	  r1 = rand() % 2; //move in x or y
	  r2 = rand() % 2; //positive or negative increment on grid
	  
	  if (r1==0) {
	    if (r2==0) {
	      x = x + cell_size;
	    } else {
	      x = x - cell_size;
	    }
	  } else {
	    if (r2==0) {
	      y = y + cell_size;
	    } else {
	      y = y - cell_size;
	    }
	  }
	  waypoints.push_back( new PositionR2( x, y ) );
	}


	// Configure turning rate and other motion parameters here
	double turning_rate = 0.2;  // rad/s
	double forward_speed = 0.5;  // m/s

	// Thresholds for switching among modes of motion
	double min_angle_err = 0.1;  // Minimum relative angle to drive forward toward waypoint
	double min_reach_err = 0.3;  // Distance from waypoint before it is declared as "reached"

	/*************************************************************/

	safe_to_move = true;
	ros::init( argc, argv, "Oscar", ros::init_options::AnonymousName );
	ros::NodeHandle nh;

	ros::Publisher action = nh.advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
	ros::Subscriber lmssub = nh.subscribe( "scan", 10, &laser_cb );
	OdomWatcher pose_watcher( nh );

	int current_index = 0;  // Visit waypoints in order
	double current_pose[3];
	double angle_diff;

	ros::Rate rate( 30. );
	
	while (ros::ok()) {

		geometry_msgs::Twist mot;

		if (safe_to_move && pose_watcher.get_pose_estimate( current_pose )) {

			angle_diff = atan2( waypoints[current_index]->x[1] - current_pose[1],
							    waypoints[current_index]->x[0] - current_pose[0] ) - current_pose[2];

			if (sqrt( (waypoints[current_index]->x[0] - current_pose[0])*(waypoints[current_index]->x[0] - current_pose[0])
					  + (waypoints[current_index]->x[1] - current_pose[1])*(waypoints[current_index]->x[1] - current_pose[1]) ) < min_reach_err) {
				std::cout << "Reached waypoint " << current_index << std::endl;
				current_index = (current_index+1) % waypoints.size();
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

		action.publish( mot );
		ros::spinOnce();
		rate.sleep();
	}


	/* It would be better to use something like Boost shared_ptr, but to avoid
	   introducing extra external dependencies, we garbage collect manually. */
	for (std::vector<PositionR2 *>::iterator it_waypt = waypoints.begin();
		 it_waypt != waypoints.end(); it_waypt++)
		delete *it_waypt;

	return 0;
}
