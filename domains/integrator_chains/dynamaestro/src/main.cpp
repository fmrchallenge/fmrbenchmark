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
#include "dynamaestro/LabelStamped.h"
#include "dynamaestro/ProblemInstanceJSON.h"
#include "dynamaestro/DMMode.h"
#endif

#include "problem.h"

#include <list>
#include <cstdlib>
#include <cstdio>
#include <errno.h>
#include <assert.h>

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
/** \ingroup integrator_chains */
class TrajectoryGenerator {
public:
	TrajectoryGenerator( int numdim_output, int highest_order_deriv );

	/* Consult the class description regarding the assumed arrangement of the
	   state variables, i.e., the indexing and the assumption that the system is
	   a chain of integrators. */
	TrajectoryGenerator( Eigen::VectorXd Xinit, int numdim_output );

	void clear();

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

private:
	Eigen::VectorXd X;
	double t;  // Current time
	int highest_order_deriv;
	int numdim_output;

	Eigen::VectorXd Xinit;
};

TrajectoryGenerator::TrajectoryGenerator( int numdim_output, int highest_order_deriv )
	: t(0.0),
	  highest_order_deriv(highest_order_deriv), numdim_output(numdim_output)
{
	X.resize( numdim_output*highest_order_deriv );
	X.setZero();
	Xinit = X;
}

TrajectoryGenerator::TrajectoryGenerator( Eigen::VectorXd Xinit, int numdim_output  )
	: t(0.0), Xinit(Xinit), X(Xinit), numdim_output(numdim_output)
{
	assert( X.size() % numdim_output == 0 );
	highest_order_deriv = X.size()/numdim_output;
}

void TrajectoryGenerator::clear()
{
	X = Xinit;
	t = 0.0;
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


/** \ingroup integrator_chains */
class TGThread {
public:
	TGThread( ros::NodeHandle &nh );
	~TGThread();
	void inputcb( const dynamaestro::VectorStamped &vs );
	bool mode_request( dynamaestro::DMMode::Request &req,
					   dynamaestro::DMMode::Response &res );
	void run();
	void pubstate( const TrajectoryGenerator &tg, Eigen::VectorXd &Y );

private:
	boost::mutex mtx_;
	bool fresh_input;
	Eigen::VectorXd U;

	Problem *probinstance;
	Labeler labeler;

	enum DMMode { ready, waiting, running, paused, resetting, restarting };
	DMMode dmmode;

	ros::NodeHandle &nh_;
	ros::ServiceServer mode_srv;
	ros::Publisher problemJSONpub;
	ros::Publisher statepub;
	ros::Publisher loutpub;
	ros::Subscriber inputsub;

	/* param_name is a name in the ROS Parameter Server. It should be a string
	   of the form "J K", where the range will be parsed as [J, K]. Return true
	   iff success. If return value is false, then the contents of \p range are
	   undefined (and may have changed). */
	bool parse_range_str( const std::string param_name, Eigen::Vector2i &range );
	bool parse_range_str( const std::string param_name, Eigen::Vector2d &range );

	/* Like parse_range_str() but for arbitrarily long arrays. If
	   expected_length is positive, then exactly that many values must be read
	   for the call to be considered successful. Otherwise (default), continue
	   to read values until no more are detected. */
	bool parse_array_str( const std::string param_name, Eigen::VectorXd &array,
						  int expected_length=-1 );
};

bool TGThread::parse_array_str( const std::string param_name, Eigen::VectorXd &array,
								const int expected_length )
{
	// Try to handle special case of singletons
	if (expected_length <= 1) {
		double singleton;
		if (nh_.getParam( param_name, singleton )) {
			array.resize( 1 );
			array(0) = singleton;
			return true;
		}
	}
	if (expected_length <= 1) {
		int singleton;
		if (nh_.getParam( param_name, singleton )) {
			array.resize( 1 );
			array(0) = singleton;
			return true;
		}
	}

	std::list<double> buffer;
	std::string array_str;
	if (!nh_.getParam( param_name, array_str ))
		return false;

	if (expected_length > 0) {
		array.resize( expected_length );
	}

	double current_value;
	char *endptr;
	errno = 0;
	current_value = strtod( array_str.data(), &endptr );
	if (errno || array_str.data() == endptr) {
		std::cerr << "dynamaestro: Malformed array string in \""
				  << param_name << "\"" << std::endl;
		return false;
	}
	if (expected_length > 0) {
		array(0) = current_value;
	} else {
		buffer.push_back( current_value );
	}

	char *prev_endptr = NULL;
	int read_count = 1;
	while (prev_endptr != endptr) {
		prev_endptr = endptr;
		errno = 0;
		current_value = strtod( prev_endptr, &endptr );
		if (errno) {
			std::cerr << "dynamaestro: Malformed array string in \""
					  << param_name << "\"" << std::endl;
			return false;
		}
		if (prev_endptr == endptr)
			break;

		if (expected_length > 0) {
			array(read_count) = current_value;
		} else {
			buffer.push_back( current_value );
		}

		read_count++;
		if (expected_length > 0 && read_count == expected_length)
			break;
	}

	if (expected_length <= 0) {
		array.resize( read_count );
		int i = 0;
		for (std::list<double>::iterator it_buff = buffer.begin();
			 it_buff != buffer.end(); it_buff++) {
			array(i) = *it_buff;
			i++;
		}
	} else if (read_count != expected_length) {
		return false;
	}

	return true;
}

bool TGThread::parse_range_str( const std::string param_name, Eigen::Vector2i &range )
{
	std::string range_str;
	if (!nh_.getParam( param_name, range_str ))
		return false;

	char *endptr;
	errno = 0;
	range(0) = strtol( range_str.data(), &endptr, 10 );
	if (errno || endptr == range_str.data()) {
		std::cerr << "dynamaestro: Malformed range string in \"" << param_name << "\"" << std::endl;
		return false;
	}
	char *prev_endptr = endptr;
	errno = 0;
	range(1) = strtol( prev_endptr, &endptr, 10 );
	if (errno || endptr == prev_endptr) {
		std::cerr << "dynamaestro: Malformed range string in \"" << param_name << "\"" << std::endl;
		return false;
	}

	return true;
}

bool TGThread::parse_range_str( const std::string param_name, Eigen::Vector2d &range )
{
	std::string range_str;
	if (!nh_.getParam( param_name, range_str ))
		return false;

	char *endptr;
	errno = 0;
	range(0) = strtod( range_str.data(), &endptr );
	if (errno || endptr == range_str.data()) {
		std::cerr << "dynamaestro: Malformed range string in \""
				  << param_name << "\"" << std::endl;
		return false;
	}
	char *prev_endptr = endptr;
	errno = 0;
	range(1) = strtod( prev_endptr, &endptr );
	if (errno || endptr == prev_endptr) {
		std::cerr << "dynamaestro: Malformed range string in \""
				  << param_name << "\"" << std::endl;
		return false;
	}

	return true;
}

TGThread::TGThread( ros::NodeHandle &nh )
	: nh_(nh),
	  fresh_input(false),
	  U(),
	  probinstance(NULL),
	  labeler(),
	  dmmode(paused)
{
	mode_srv = nh_.advertiseService( "mode", &TGThread::mode_request, this );
	problemJSONpub = nh_.advertise<dynamaestro::ProblemInstanceJSON>( "probleminstance_JSON", 1, true );
	statepub = nh_.advertise<dynamaestro::VectorStamped>( "state", 10, true );
	loutpub = nh_.advertise<dynamaestro::LabelStamped>( "loutput", 10, true );
	inputsub = nh_.subscribe( "input", 1, &TGThread::inputcb, this );
}

TGThread::~TGThread()
{
	if (probinstance)
		delete probinstance;
}

void TGThread::pubstate( const TrajectoryGenerator &tg, Eigen::VectorXd &Y )
{
	dynamaestro::VectorStamped pt;
	pt.header.frame_id = std::string( "map" );
	pt.header.stamp = ros::Time::now();
	for (int i = 0; i < tg.getStateDim(); i++) {
		pt.v.point.push_back( tg.getState( i ) );
		if (i < tg.getOutputDim())
			Y(i) = tg[i];
	}
	statepub.publish( pt );

	std::list<std::string> label = labeler.get_label( Y );
	dynamaestro::LabelStamped lbl;
	lbl.header.frame_id = pt.header.frame_id;
	lbl.header.stamp = pt.header.stamp;
	lbl.label.resize( label.size() );
	int i = 0;
	for (std::list<std::string>::iterator it_label = label.begin();
		 it_label != label.end(); it_label++) {
		lbl.label[i] = *it_label;
		i++;
	}
	loutpub.publish( lbl );
}

bool TGThread::mode_request( dynamaestro::DMMode::Request &req,
							 dynamaestro::DMMode::Response &res )
{
	switch (req.mode) {
	case dynamaestro::DMMode::Request::UNPAUSE:
		if (dmmode != running)
			dmmode = running;
		res.result = true;
		break;

	case dynamaestro::DMMode::Request::READY:
		if (dmmode == ready) {
			res.result = true;
		} else {
			res.result = false;
		}
		break;

	case dynamaestro::DMMode::Request::START:
		if (dmmode == ready) {
			dmmode = waiting;
			res.result = true;
		} else {
			res.result = false;
		}
		break;

	case dynamaestro::DMMode::Request::PAUSE:
		if (dmmode == running) {
			dmmode = paused;
			fresh_input = false;
		}
		res.result = true;
		break;

	case dynamaestro::DMMode::Request::RESTART:
		dmmode = restarting;
		ROS_INFO( "dynamaestro: Restarting trial..." );
		res.result = true;
		break;

	case dynamaestro::DMMode::Request::RESET:
		dmmode = resetting;
		ROS_INFO( "dynamaestro: Stopping current trial and generating a new instance..." );
		res.result = true;
		break;

	default:
		return false;
	}

	return true;
}

void TGThread::inputcb( const dynamaestro::VectorStamped &vs )
{
	assert( probinstance != NULL );

	mtx_.lock();

	fresh_input = true;
	for (int i = 0; i < probinstance->get_numdim_output(); i++)
		U[i] = vs.v.point[i];

	mtx_.unlock();
}

void TGThread::run()
{
	Eigen::Vector2i duration_bounds;
	if (!parse_range_str( "duration_bounds", duration_bounds )) {
		duration_bounds << 30, 90;
	}
	ROS_INFO( "dynamaestro: Using [%d, %d] as range of integers for sampling "
			  "the trial duration.",
			  duration_bounds(0), duration_bounds(1) );

	int nominal_duration = duration_bounds(0);
	if (duration_bounds(0) == duration_bounds(1))
		nominal_duration += rand() % (1+duration_bounds(1)-duration_bounds(1));

	Eigen::Vector2i numdim_output_bounds;
	if (!parse_range_str( "output_dim_bounds", numdim_output_bounds )) {
		numdim_output_bounds << 1, 3;
	}
	ROS_INFO( "dynamaestro: Using [%d, %d] as sampling range for number of "
			  "dimensions of the output space.",
			  numdim_output_bounds(0), numdim_output_bounds(1) );

	Eigen::Vector2i num_integrators_bounds;
	if (!parse_range_str( "number_integrators_bounds", num_integrators_bounds )) {
		num_integrators_bounds << 1, 3;
	}
	ROS_INFO( "dynamaestro: Using [%d, %d] as sampling range for number of "
			  "integrators.",
			  num_integrators_bounds(0), num_integrators_bounds(1) );

	Eigen::Vector2i number_goals_bounds;
	if (!parse_range_str( "number_goals_bounds", number_goals_bounds )) {
		number_goals_bounds << 1, 4;
	}
	ROS_INFO( "dynamaestro: Using [%d, %d] as sampling range for number of "
			  "goal polytopes.",
			  number_goals_bounds(0), number_goals_bounds(1) );

	Eigen::Vector2i number_obstacles_bounds;
	if (!parse_range_str( "number_obstacles_bounds", number_obstacles_bounds )) {
		number_obstacles_bounds << 0, 4;
	}
	ROS_INFO( "dynamaestro: Using [%d, %d] as sampling range for number of "
			  "obstacle polytopes.",
			  number_obstacles_bounds(0), number_obstacles_bounds(1) );

	Eigen::VectorXd Y_box;
	if (!parse_array_str( "Y", Y_box )
		|| (Y_box.size() > 1 && Y_box.size() < numdim_output_bounds(1)*2)) {
		Y_box.resize( 2*numdim_output_bounds(1) );
		for (int i = 0; i < numdim_output_bounds(1); i++) {
			Y_box(2*i+1) = 10;
			Y_box(2*i) = -Y_box(2*i+1);
		}
	} else if (Y_box.size() == 1) {
		double Y_ref = Y_box(0);
		Y_box.resize( 2*numdim_output_bounds(1) );
		for (int i = 0; i < numdim_output_bounds(1); i++) {
			Y_box(2*i+1) = Y_ref;
			Y_box(2*i) = -Y_box(2*i+1);
		}
	}

	Eigen::VectorXd U_box;
	if (!parse_array_str( "U", U_box )
		|| (U_box.size() > 1 && U_box.size() < numdim_output_bounds(1)*2)) {
		U_box.resize( 2*numdim_output_bounds(1) );
		for (int i = 0; i < numdim_output_bounds(1); i++) {
			U_box(2*i+1) = 1;
			U_box(2*i) = -U_box(2*i+1);
		}
	} else if (U_box.size() == 1) {
		double U_ref = U_box(0);
		U_box.resize( 2*numdim_output_bounds(1) );
		for (int i = 0; i < numdim_output_bounds(1); i++) {
			U_box(2*i+1) = U_ref;
			U_box(2*i) = -U_box(2*i+1);
		}
	}

	Eigen::VectorXd Xinit_bounds;

	Eigen::Vector2d period_bounds;
	if (!parse_range_str( "period_bounds", period_bounds )) {
		period_bounds << 0.01, 0.05;
	}
	ROS_INFO( "dynamaestro: Using [%f, %f] as sampling range for number of "
			  "discretization period.",
			  period_bounds(0), period_bounds(1) );

	probinstance = Problem::random( numdim_output_bounds,
									num_integrators_bounds,
									Y_box, U_box,
									number_goals_bounds,
									number_obstacles_bounds,
									period_bounds,
									Y_box );
	if (probinstance->get_highest_order_deriv() > 1)
		probinstance->Xinit.tail( (probinstance->get_highest_order_deriv()-1)*probinstance->get_numdim_output() ).setZero();
	labeler.importProblem( *probinstance );

	U.resize( probinstance->get_numdim_output() );
	U.setZero();

	Eigen::VectorXd Y( probinstance->get_numdim_output() );

	TrajectoryGenerator tg( probinstance->Xinit,
							probinstance->get_numdim_output() );

	ros::Rate rate( 1/probinstance->period );
	Eigen::VectorXd defaultU( U );
	defaultU.setZero();

	ros::Duration trial_duration;

	ros::Rate polling_rate( 100 );
	dmmode = ready;
	while (dmmode == ready) {
		if (!ros::ok())
			return;
		ros::spinOnce();
		polling_rate.sleep();
	}
	assert( dmmode == waiting );
	ros::Time startt = ros::Time::now();

	dynamaestro::ProblemInstanceJSON probinstance_msg;
	probinstance_msg.stamp = ros::Time::now();
	probinstance_msg.problemjson = probinstance->dumpJSON();
	problemJSONpub.publish( probinstance_msg );

	nh_.setParam( "number_integrators",
				  probinstance->get_highest_order_deriv() );
	ROS_INFO( "dynamaestro: Using %d as number of integrators.",
			  probinstance->get_highest_order_deriv() );

	nh_.setParam( "output_dim", probinstance->get_numdim_output() );
	ROS_INFO( "dynamaestro: Using %d as dimension of the output space.",
			  probinstance->get_numdim_output() );

	nh_.setParam( "period", probinstance->period );
	ROS_INFO( "dynamaestro: Using %f seconds as the period.",
			  probinstance->period );

	// Send initial state, before any input is applied or time has begun.
	pubstate( tg, Y );
	ros::spinOnce();

	while (ros::ok() && dmmode != resetting && (trial_duration = ros::Time::now() - startt).toSec() < nominal_duration) {
		if (dmmode == running) {
			if (fresh_input) {
				mtx_.lock();

				fresh_input = false;
				tg.step( probinstance->period, U );

				mtx_.unlock();
			} else {
				tg.step( probinstance->period, defaultU );
			}

			pubstate( tg, Y );

		} else if (dmmode == restarting) {
			dmmode = paused;
			mtx_.lock();
			fresh_input = false;
			mtx_.unlock();
			tg.clear();
			U.setZero();
			pubstate( tg, Y );
			dmmode = waiting;
		} else if (dmmode == waiting) {
			if (fresh_input) {
				dmmode = running;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	dmmode = resetting;
	dynamaestro::VectorStamped pt;
	// Send empty state message to indicate that trial has ended.
	statepub.publish( pt );

	// Did the trial end because the nominal (hidden) duration was reached?
	if (ros::ok() && trial_duration.toSec() >= nominal_duration) {
		ROS_INFO( "dynamaestro: Trial ended after %f s (threshold is %d s).", trial_duration.toSec(), nominal_duration );
	}

	// Clear registered instance
	ROS_INFO( "dynamaestro: Clearing instance summary from Parameter Server." );
	nh_.setParam( "number_integrators", -1 );
	nh_.setParam( "output_dim", -1 );
	nh_.setParam( "period", -1.0 );
	fresh_input = false;
	labeler.clear();
	delete probinstance;
}

void tgthread( ros::NodeHandle &nhp )
{
	TGThread tgt( nhp );
	while (ros::ok()) {
		tgt.run();
	}
}


int main( int argc, char **argv )
{
#ifdef USE_ROS
	ros::init( argc, argv, "dynamaestro" );
	ros::NodeHandle nh( "~" );

	time_t seed = time( NULL );
	srand( seed );
	ROS_INFO( "dynamaestro: Using %ld as the PRNG seed.", seed );

	boost::thread *tgmain = NULL;
	tgmain = new boost::thread( tgthread, nh );
	tgmain->join();
	delete tgmain;

	ros::spin();
	
#endif
	return 0;
}
