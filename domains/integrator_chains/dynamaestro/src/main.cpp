/* main.cpp - main entry point for the command-line tool `dm`
 *
 * The current implementation is specific to chains of integrators;
 * however, the design is intended to be general to facilitate later
 * treatment of other kinds of systems.
 *
 * While some infrastructure is in place for building without ROS, it
 * is as yet not operational.
 *
 *
 * SCL; Jan 2015
 */


#ifdef USE_ROS
#include <ros/ros.h>
#include <dynamaestro/VectorStamped.h>
#endif

#include <cmath>
#include <vector>

#include <pthread.h>


class TrajectoryGenerator {
private:
	std::vector<double> X;
	double t;  // Current time
	int numdim_input;
	int numdim_output;

public:
	/* Because we are assuming the chain of integrators system, the
	   number of dimensions of the input (parameter numdim_input to
	   the constructors) must equal the number of dimensions of the
	   output (parameter numdim_output). */
	TrajectoryGenerator( int numdim_state, int numdim_input, int numdim_output );
	TrajectoryGenerator( std::vector<double> Xinit, int numdim_input, int numdim_output );

	double step( double dt, const double &U );
	double getTime() const
		{ return t; }
	double getStateDim() const
		{ return X.size(); }

	// Output vector
	double operator[]( int i ) const;

	/* Access to full, current state vector */
	double getState( int i ) const
		{ return X[i]; }
};

TrajectoryGenerator::TrajectoryGenerator( int numdim_state, int numdim_input, int numdim_output )
	: t(0.0), numdim_input(numdim_input), numdim_output(numdim_output)
{
	for (int i = 0; i < numdim_state; i++)
		X.push_back( 5.0 );
}

TrajectoryGenerator::TrajectoryGenerator( std::vector<double> Xinit, int numdim_input, int numdim_output  )
	: t(0.0), X(Xinit), numdim_input(numdim_input), numdim_output(numdim_output)
{ }

double TrajectoryGenerator::step( double dt, const double &U )
{
	int numdim_state = X.size();

	for (int i = 0; i <= numdim_state-2; i++)
		X[i] += dt*X[i+1];

	X[numdim_state-1] += dt*U;

	t += dt;
	return t;
}

double TrajectoryGenerator::operator[]( int i ) const
{
	if (i < numdim_output) {
		return X[i];
	} else {
		ROS_ERROR( "Invalid index into output vector: %d", i );
		return 0.0/0.0;
	}
}


/* nhp should point to an instance of ros::NodeHandle */
void *tgthread( void *nhp )
{
	ros::Publisher statepub = ((ros::NodeHandle *)nhp)->advertise<dynamaestro::VectorStamped>( "output", 10 );

	double h = 0.1;  // Sampling period
	TrajectoryGenerator tg( 3, 1, 1 );

	// Send initial output, before any input is applied or time has begun.
	dynamaestro::VectorStamped pt;
	pt.header.frame_id = std::string( "map" );
	pt.header.stamp = ros::Time::now();
	for (int i = 0; i < tg.getStateDim(); i++)
		pt.point.push_back( tg.getState( i ) );
	statepub.publish( pt );
	ros::spinOnce();

	ros::Rate rate( 1/h );
	while (ros::ok()) {
		tg.step( h, -(tg.getState( 0 ) + 2.4142*tg.getState( 1 ) + 2.4142*tg.getState( 2 )) );

		dynamaestro::VectorStamped pt;
		pt.header.frame_id = std::string( "map" );
		pt.header.stamp = ros::Time::now();
		for (int i = 0; i < tg.getStateDim(); i++)
			pt.point.push_back( tg.getState( i ) );
		statepub.publish( pt );
		ros::spinOnce();
		rate.sleep();
	}
}


int main( int argc, char **argv )
{
#ifdef USE_ROS
	ros::init( argc, argv, "dynamaestro" );
	ros::NodeHandle nh;

	pthread_t tgID;
	if (pthread_create( &tgID, NULL, tgthread, (void *)&nh )) {
		perror( "dm, pthread_create" );
		return -1;
	}

	ros::Rate rate( 10. );
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	
#endif
	return 0;
}
