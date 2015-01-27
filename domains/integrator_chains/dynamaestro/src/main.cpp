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

#include <cstdlib>
#include <cstdio>
#include <vector>

#include <pthread.h>
#include <semaphore.h>


/* Because we are assuming the chain of integrators system, the number of
   dimensions of the input (parameter numdim_input to the constructors) must
   equal the number of dimensions of the output (parameter numdim_output).

   The trajectories are the solutions of linear time-invariant control
   system. Control input is applied using a zero-order hold, i.e., applied
   constantly during the duration given as the first parameter of step().

   Let M be the highest order of derivative, and let N be the dimension of the
   output space.  The state variable indexing is such that the first output
   variable is the first state variable, the second output variable is the
   second state variable, etc. Thus the component systems are interleaved in the
   sense that the first subsystem is formed from state variable indices 1, 1+N,
   1+2N, ..., 1+(M-1)N, and the input to this subsystem is applied at state
   variable index 1+(M-1)N.
*/
class TrajectoryGenerator {
private:
	std::vector<double> X;
	double t;  // Current time
	int highest_order_deriv;
	int numdim_output;

public:
	TrajectoryGenerator( int numdim_output, int highest_order_deriv );

	/* Consult the class description regarding the assumed arrangement of the
	   state variables, i.e., the indexing and the assumption that the system is
	   a chain of integrators. */
	TrajectoryGenerator( std::vector<double>Xinit, int numdim_output );

	/* Forward Euler integration for duration dt using constant input U */
	double step( double dt, const std::vector<double> &U );

	double getTime() const
		{ return t; }
	double getStateDim() const
		{ return X.size(); }
	double getOutputDim() const
		{ return numdim_output; }

	// Output vector
	double operator[]( int i ) const;

	/* Access to full, current state vector */
	double getState( int i ) const
		{ return X[i]; }
};

TrajectoryGenerator::TrajectoryGenerator( int numdim_output, int highest_order_deriv )
	: t(0.0), highest_order_deriv(highest_order_deriv), numdim_output(numdim_output)
{
	for (int i = 0; i < numdim_output*highest_order_deriv; i++)
		X.push_back( 5.0 );
}

TrajectoryGenerator::TrajectoryGenerator( std::vector<double> Xinit, int numdim_output  )
	: t(0.0), X(Xinit), numdim_output(numdim_output)
{
	highest_order_deriv = X.size()/numdim_output;  // N.B., assumed to divide evenly!
}

double TrajectoryGenerator::step( double dt, const std::vector<double> &U )
{
	if (U.size() > numdim_output) {
		ROS_ERROR( "Input vector has too many elements." );
		return 0.0/0.0;
	}

	int numdim_state = X.size();

	int i, j;
	for (j = 0; j < numdim_output; j++) {
		for (i = 0; i <= highest_order_deriv-2; i++) {
			X[i*numdim_output+j] += dt*X[(i+1)*numdim_output+j];
		}
		X[(highest_order_deriv-1)*numdim_output+j] += dt*U[j];
	}

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


class TGThread {
private:
	sem_t sem;
	bool fresh_input;
	std::vector<double> U;

	int numdim_output;
	int highest_order_deriv;
	double h;  // Sampling period

public:
	TGThread( int numdim_output = 0, int highest_order_deriv = 0, double period = 0.0 );
	~TGThread();
	void inputcb( const dynamaestro::VectorStamped &vs );
	void run( ros::NodeHandle &nh );
};

TGThread::TGThread( int numdim_output, int highest_order_deriv, double period )
	: fresh_input(false),
	  numdim_output(numdim_output), highest_order_deriv(highest_order_deriv),
	  h(period)
{
	if (sem_init( &sem, 0, 1 )) {
		perror( "TGThread, sem_init" );
		exit( -1 );
	}

	for (int i; i < numdim_output; i++)
		U.push_back( 0.0 );
}

TGThread::~TGThread()
{
	if (sem_destroy( &sem )) {
		perror( "~TGThread, sem_destroy" );
		exit( -1 );
	}
}

void TGThread::inputcb( const dynamaestro::VectorStamped &vs )
{
	if (sem_wait( &sem )) {
		perror( "inputcb, sem_wait" );
		exit( -1 );
	}

	fresh_input = true;
	U = vs.point;

	if (sem_post( &sem )) {
		perror( "inputcb, sem_post" );
		exit( -1 );
	}
}

void TGThread::run( ros::NodeHandle &nh )
{
	ros::Publisher statepub = nh.advertise<dynamaestro::VectorStamped>( "output", 10 );
	ros::Subscriber inputsub = nh.subscribe( "input", 1, &TGThread::inputcb, this );


	TrajectoryGenerator tg( numdim_output, highest_order_deriv );

	ros::Rate rate( 1/h );
	std::vector<double> defaultU;
	for (int i = 0; i < numdim_output; i++)
		defaultU.push_back( 0.0 );

	// Send initial output, before any input is applied or time has begun.
	dynamaestro::VectorStamped pt;
	pt.header.frame_id = std::string( "map" );
	pt.header.stamp = ros::Time::now();
	for (int i = 0; i < tg.getStateDim(); i++)
		pt.point.push_back( tg.getState( i ) );
	statepub.publish( pt );
	ros::spinOnce();

	while (ros::ok()) {
		if (fresh_input) {
			if (sem_wait( &sem )) {
				perror( "TGThread::run, sem_wait" );
				exit( -1 );
			}

			fresh_input = false;
			tg.step( h, U );

			if (sem_post( &sem )) {
				perror( "TGThread::run, sem_post" );
				exit( -1 );
			}
		} else {
			tg.step( h, defaultU );
		}

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

/* nhp should point to an instance of ros::NodeHandle */
void *tgthread( void *nhp )
{
	TGThread tgt( 1, 3, 0.1 );
	tgt.run( *(ros::NodeHandle *)nhp );
}


int main( int argc, char **argv )
{
	pthread_t tgID;

#ifdef USE_ROS
	ros::init( argc, argv, "dynamaestro" );
	ros::NodeHandle nh;

	if (pthread_create( &tgID, NULL, tgthread, (void *)&nh )) {
		perror( "dm, pthread_create" );
		exit( -1 );
	}

	ros::Rate rate( 10. );
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	
#endif
	return 0;
}
