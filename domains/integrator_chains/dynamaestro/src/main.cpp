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
 * SCL; 2015
 */


#ifdef USE_ROS
#include <ros/ros.h>
#include "dynamaestro/VectorStamped.h"
#include "dynamaestro/DMMode.h"
#endif

#include <cstdlib>
#include <cstdio>

#include <Eigen/Dense>
#include <boost/thread/thread.hpp>


/* The trajectories are the solutions of a linear time-invariant control
   system. Control input is applied using a zero-order hold, i.e., applied
   constantly during the duration given as the first parameter of step().

   Let m be the highest order of derivative, and let n be the dimension of the
   output space.  The state variable indexing is such that the first output
   variable is the first state variable, the second output variable is the
   second state variable, etc. Thus the component systems are interleaved in the
   sense that the first subsystem is formed from state variable indices 1, 1+n,
   1+2n, ..., 1+(m-1)n, and the input to this subsystem is applied at state
   variable index 1+(m-1)n.
*/
class TrajectoryGenerator {
private:
	Eigen::VectorXd X;
	double t;  // Current time
	int highest_order_deriv;
	int numdim_output;

public:
	TrajectoryGenerator( int numdim_output, int highest_order_deriv );

	/* Consult the class description regarding the assumed arrangement of the
	   state variables, i.e., the indexing and the assumption that the system is
	   a chain of integrators. */
	TrajectoryGenerator( Eigen::VectorXd Xinit, int numdim_output );

	/* Forward Euler integration for duration dt using constant input U */
	double step( double dt, const Eigen::VectorXd &U );

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
	: t(0.0),
	  highest_order_deriv(highest_order_deriv), numdim_output(numdim_output)
{
	X.resize( numdim_output*highest_order_deriv );
	X.setZero();
}

TrajectoryGenerator::TrajectoryGenerator( Eigen::VectorXd Xinit, int numdim_output  )
	: t(0.0), X(Xinit), numdim_output(numdim_output)
{
	highest_order_deriv = X.size()/numdim_output;  // N.B., assumed to divide evenly!
}

double TrajectoryGenerator::step( double dt, const Eigen::VectorXd &U )
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
	boost::mutex mtx_;
	bool fresh_input;
	Eigen::VectorXd U;

	int numdim_output;
	int highest_order_deriv;
	double h;  // Sampling period

	bool running;

	ros::NodeHandle &nh_;
	ros::ServiceServer mode_srv;

public:
	TGThread( ros::NodeHandle &nh, int numdim_output = 1, int highest_order_deriv = 2, double period = 0.1 );
	~TGThread();
	void inputcb( const dynamaestro::VectorStamped &vs );
	bool mode_request( dynamaestro::DMMode::Request &req,
					   dynamaestro::DMMode::Response &res );
	void run();
};

TGThread::TGThread( ros::NodeHandle &nh, int numdim_output, int highest_order_deriv, double period )
	: nh_(nh),
	  fresh_input(false),
	  numdim_output(numdim_output), highest_order_deriv(highest_order_deriv),
	  h(period), U(numdim_output),
	  running(false)
{
	mode_srv = nh_.advertiseService( "mode", &TGThread::mode_request, this );
}

TGThread::~TGThread()
{ }

bool TGThread::mode_request( dynamaestro::DMMode::Request &req,
							 dynamaestro::DMMode::Response &res )
{
	switch (req.mode) {
	case dynamaestro::DMMode::Request::UNPAUSE:
		if (!running)
			running = true;
		res.result = true;
		break;

	case dynamaestro::DMMode::Request::PAUSE:
		if (running) {
			running = false;
			fresh_input = false;
		}
		res.result = true;
		break;

	default:
		return false;
	}

	return true;
}

void TGThread::inputcb( const dynamaestro::VectorStamped &vs )
{
	mtx_.lock();

	fresh_input = true;
	for (int i = 0; i < numdim_output; i++)
		U[i] = vs.v.point[i];

	mtx_.unlock();
}

void TGThread::run()
{
	ros::Publisher statepub = nh_.advertise<dynamaestro::VectorStamped>( "output", 10, true );
	ros::Subscriber inputsub = nh_.subscribe( "input", 1, &TGThread::inputcb, this );

	nh_.getParam( "number_integrators", highest_order_deriv );
	ROS_INFO( "dynamaestro: Using %d as number of integrators.", highest_order_deriv );

	nh_.getParam( "output_dim", numdim_output );
	ROS_INFO( "dynamaestro: Using %d as dimension of the output space.", numdim_output );

	nh_.getParam( "period", h );
	ROS_INFO( "dynamaestro: Using %f seconds as the period.", h );

	U.resize( numdim_output );
	U.setZero();

	TrajectoryGenerator tg( numdim_output, highest_order_deriv );

	ros::Rate rate( 1/h );
	Eigen::VectorXd defaultU( U );
	defaultU.setZero();

	// Send initial output, before any input is applied or time has begun.
	dynamaestro::VectorStamped pt;
	pt.header.frame_id = std::string( "map" );
	pt.header.stamp = ros::Time::now();
	for (int i = 0; i < tg.getStateDim(); i++)
		pt.v.point.push_back( tg.getState( i ) );
	statepub.publish( pt );
	ros::spinOnce();

	while (ros::ok()) {
		if (running) {
			if (fresh_input) {
				mtx_.lock();

				fresh_input = false;
				tg.step( h, U );

				mtx_.unlock();
			} else {
				tg.step( h, defaultU );
			}

			dynamaestro::VectorStamped pt;
			pt.header.frame_id = std::string( "map" );
			pt.header.stamp = ros::Time::now();
			for (int i = 0; i < tg.getStateDim(); i++)
				pt.v.point.push_back( tg.getState( i ) );
			statepub.publish( pt );
		}
		ros::spinOnce();
		rate.sleep();
	}
}

void tgthread( ros::NodeHandle &nhp )
{
	TGThread tgt( nhp );
	tgt.run();
}


int main( int argc, char **argv )
{
#ifdef USE_ROS
	ros::init( argc, argv, "dynamaestro" );
	ros::NodeHandle nh( "~" );

	boost::thread tgmain( tgthread, nh );

	ros::Rate rate( 10. );
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	tgmain.join();
	
#endif
	return 0;
}
