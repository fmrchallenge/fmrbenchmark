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

#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <list>
#include <cstdlib>
#include <cstdio>
#include <errno.h>
#include <assert.h>

#include <Eigen/Dense>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


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


template <typename T>
class Queue {
public:
	void enqueue( int, T );
	std::pair<int, T> dequeue();
	int size();
	int peek();
private:
	boost::mutex mtx_;
	std::list< std::pair<int, T> > Q;
};

template <typename T>
void Queue<T>::enqueue( int k, T obj )
{
	mtx_.lock();
	Q.push_front( std::pair<int, T>( k, obj ) );
	mtx_.unlock();
}

template <typename T>
std::pair<int, T> Queue<T>::dequeue()
{
	assert( Q.size() > 0 );
	mtx_.lock();
	std::pair<int, T> x = Q.back();
	Q.pop_back();
	mtx_.unlock();
	return x;
}

template <typename T>
int Queue<T>::size()
{
	mtx_.lock();
	int sz = Q.size();
	mtx_.unlock();
	return sz;
}

template <typename T>
int Queue<T>::peek()
{
	assert( this->size() > 0 );
	mtx_.lock();
	int k = Q.back().first;
	mtx_.unlock();
	return k;
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

	void inc_trial();
	int get_trial_number();

	/* Spin up a scribe thread to record trial data.

	   The scribe thread is responsible for freeing each message in the state
	   and problem instance queues. It can be terminated using stop_monitoring()

	   Other assumptions: All of the following assumptions ignore the case of
	   thread interruption, e.g., due to stop_monitoring() or ROS shutdown.
	   Interruption causes the superseding assumption to be made that no more
	   messages will arrive and everything from the queues is safe to delete.
	   Upon arrival of an item associated with trial number t+1, all items
	   associated with trial t are safe to be freed. The arrival of the problem
	   instance description for trial t+1 implies that no more state nor input
	   items will be enqueued for trial t. The first state message should not be
	   recorded because it is redundant with the Xinit in the problem instance
	   description. Thus ignoring the first state message, the numbers of inputs
	   and remaining states for the current trial are equal unless the first
	   input is empty (vector of zero length), in which case the controller has
	   declared "unrealizable". For each trial, the problem instance and any
	   compromising details should not be recorded until after the controller
	   has begun the trial. The problem instance of trial t is safe to be freed
	   by the scribe thread only if the problem instance of trial t+1 has
	   arrived (consistent with an assumption above) or monitoring has ended. */
	void start_monitoring( std::string filename, bool append_mode=false );

	/* No-op if no scribe is active. */
	void stop_monitoring();

private:
	boost::mutex trial_mtx_;
	boost::mutex mtx_;
	bool fresh_input;
	Eigen::VectorXd U;

	Problem *probinstance;
	Labeler labeler;

	/* -1 indicates uninitialized. Counting begins at 0. */
	int trial_number;

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
	bool parse_array_str( const std::string &param_name, Eigen::VectorXd &array,
						  int expected_length=-1 );

	Queue<ros::Time> times_recording;
	Queue<Problem *> instances_recording;
	Queue<Eigen::VectorXd> inputs_recording;
	Queue<dynamaestro::VectorStamped *> states_recording;
	boost::thread *scribethread;
	void tgt_scribe( std::string filename, bool append_mode );
};

void TGThread::tgt_scribe( std::string filename, bool append_mode )
{
	std::ofstream outf( filename.c_str(),
						std::ios_base::out
						| (append_mode ?
						   std::ios_base::app : std::ios_base::trunc) );
	bool first_trial = true;
	std::pair<int, Problem *> tp( -1, NULL );

	ros::Rate polling_rate( 100 );
	while (get_trial_number() < 0 && ros::ok())
		polling_rate.sleep();

	ROS_INFO( "dynamaestro: Saving results to %s", filename.c_str() );
	outf << "\"trials\": [\n" << std::endl;
	while (!((get_trial_number() < 0 || !ros::ok()
			  || boost::this_thread::interruption_requested())
			 && instances_recording.size() == 0
			 && states_recording.size() == 0
			 && inputs_recording.size() == 0)) {

		if (instances_recording.size() == 0) {
			polling_rate.sleep();
			continue;
		}

		if (tp.second)
			delete tp.second;
		std::pair<int, Problem *> tp = instances_recording.dequeue();
		std::pair<int, ros::Time> tdurationt = times_recording.dequeue();
		assert( tdurationt.first == tp.first && tdurationt.second.nsec == 0 );

		// Wait for first state to be sure that it is safe to begin recording
		while (ros::ok() && get_trial_number() >= 0
			   && states_recording.size() == 0
			   && instances_recording.size() == 0)
			polling_rate.sleep();

		// Instance generated and queued but never revealed to the controller?
		if (states_recording.size() == 0 || states_recording.peek() != tp.first)
			continue;

		std::pair<int, ros::Time> tstartt = times_recording.dequeue();
		assert( tstartt.first == tp.first );

		if (first_trial) {
			first_trial = false;
		} else {
			outf << "," << std::endl;
		}
		outf << "{\n  \"duration\": " << tdurationt.second.sec << ",\n"
			 << "  \"start_time\": [" << tstartt.second.sec
			 << ", " << tstartt.second.nsec << "],\n"
			 << "  \"problem_instance\": " << *tp.second;

		// Wait for first input to get realizability declaration
		while (ros::ok() && get_trial_number() >= 0
			   && inputs_recording.size() == 0
			   && instances_recording.size() == 0)
			polling_rate.sleep();

		// Did the controller time-out, and the dm progress to the next trial?
		if (inputs_recording.size() == 0 || inputs_recording.peek() != tp.first) {
			outf << "}" << std::endl;
			continue;
		}

		// Drop initial state because it is already in the instance description
		states_recording.dequeue();

		bool at_least_one = false;
		while ((inputs_recording.size() > 0
				&& inputs_recording.peek() == tp.first)
			   || (ros::ok() && get_trial_number() >= 0
				   && !boost::this_thread::interruption_requested()
				   && instances_recording.size() == 0)) {

			if (inputs_recording.size() == 0 || states_recording.size() == 0) {
				polling_rate.sleep();
				continue;
			}

			std::pair<int, Eigen::VectorXd> tinput = inputs_recording.dequeue();
			if (!at_least_one) {
				std::pair<int, ros::Time> tdecisiont = times_recording.dequeue();
				assert( tdecisiont.first == tinput.first );
				outf << ",\n  \"decision_time\": [" << tdecisiont.second.sec
					 << ", " << tdecisiont.second.nsec << "]," << std::endl;

				if (tinput.second.size() == 0) {
					outf << "  \"realizable\": false" << std::endl;
					break;
				} else {
					outf << "  \"realizable\": true," << std::endl;
				}
				at_least_one = true;
				outf << "  \"trajectory\": [" << std::endl;

			} else {
				outf << "," << std::endl;
			}

			std::pair<int, dynamaestro::VectorStamped *> tstate = states_recording.dequeue();

			outf << "[" << tstate.second->header.stamp.sec
				 << ", " << tstate.second->header.stamp.nsec;
			for (int i = 0; i < tinput.second.size(); i++)
				outf << ", " << tinput.second(i);
			for (int i = 0; i < tstate.second->v.point.size(); i++)
				outf << ", " << tstate.second->v.point[i];
			outf << "]";
			delete tstate.second;

		}
		if (at_least_one)
			outf << "]" << std::endl;

		outf << "}";

	}
	outf << "\n]" << std::endl;
	outf.close();
	if (tp.second)
		delete tp.second;
}

void TGThread::start_monitoring( std::string filename, bool append_mode )
{
	assert( scribethread == NULL );
	scribethread = new boost::thread( boost::bind( &TGThread::tgt_scribe,
												   this, _1, append_mode ),
									  filename );
}

void TGThread::stop_monitoring()
{
	if (scribethread) {
		scribethread->interrupt();
		ROS_INFO( "dynamaestro: Waiting for scribe thread to finish..." );
		scribethread->join();
		delete scribethread;
		scribethread = NULL;
	}
}

bool TGThread::parse_array_str( const std::string &param_name, Eigen::VectorXd &array,
								int expected_length )
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
		std::cerr << "dynamaestro: Malformed range string in \""
				  << param_name << "\"" << std::endl;
		return false;
	}
	char *prev_endptr = endptr;
	errno = 0;
	range(1) = strtol( prev_endptr, &endptr, 10 );
	if (errno || endptr == prev_endptr) {
		std::cerr << "dynamaestro: Malformed range string in \""
				  << param_name << "\"" << std::endl;
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

int TGThread::get_trial_number()
{
	trial_mtx_.lock();
	int x = trial_number;
	trial_mtx_.unlock();
	return x;
}

void TGThread::inc_trial()
{
	trial_mtx_.lock();
	trial_number += 1;
	trial_mtx_.unlock();
}

TGThread::TGThread( ros::NodeHandle &nh )
	: nh_(nh),
	  fresh_input(false),
	  U(),
	  probinstance(NULL),
	  labeler(),
	  trial_number(-1),
	  dmmode(paused),
	  scribethread(NULL)
{
	mode_srv = nh_.advertiseService( "mode", &TGThread::mode_request, this );
	problemJSONpub = nh_.advertise<dynamaestro::ProblemInstanceJSON>( "probleminstance_JSON", 1, true );
	statepub = nh_.advertise<dynamaestro::VectorStamped>( "state", 10, true );
	loutpub = nh_.advertise<dynamaestro::LabelStamped>( "loutput", 10, true );
	inputsub = nh_.subscribe( "input", 1, &TGThread::inputcb, this );
}



TGThread::~TGThread()
{
	if (scribethread) {
		stop_monitoring();
	} else if (probinstance) {
		delete probinstance;
	}
}

void TGThread::pubstate( const TrajectoryGenerator &tg, Eigen::VectorXd &Y )
{
	dynamaestro::VectorStamped *pt = new dynamaestro::VectorStamped();
	pt->header.frame_id = std::string( "map" );
	pt->header.stamp = ros::Time::now();
	for (int i = 0; i < tg.getStateDim(); i++) {
		pt->v.point.push_back( tg.getState( i ) );
		if (i < tg.getOutputDim())
			Y(i) = tg[i];
	}
	statepub.publish( *pt );

	std::list<std::string> label = labeler.get_label( Y );
	dynamaestro::LabelStamped lbl;
	lbl.header.frame_id = pt->header.frame_id;
	lbl.header.stamp = pt->header.stamp;
	lbl.label.resize( label.size() );
	int i = 0;
	for (std::list<std::string>::iterator it_label = label.begin();
		 it_label != label.end(); it_label++) {
		lbl.label[i] = *it_label;
		i++;
	}
	loutpub.publish( lbl );

	if (scribethread) {
		states_recording.enqueue( get_trial_number(), pt );
	} else {
		delete pt;
	}
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
	inc_trial();
	bool realizable;  // To be decided by the controller

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

	if (scribethread) {
		times_recording.enqueue( get_trial_number(),
								 ros::Time( nominal_duration, 0 ) );
		instances_recording.enqueue( get_trial_number(), probinstance );
	}

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

	dynamaestro::ProblemInstanceJSON probinstance_msg;
	probinstance_msg.stamp = ros::Time::now();
	probinstance_msg.problemjson = probinstance->dumpJSON();
	problemJSONpub.publish( probinstance_msg );
	ros::Time startt( probinstance_msg.stamp.sec,
					  probinstance_msg.stamp.nsec );
	if (scribethread)
		times_recording.enqueue( get_trial_number(), startt );

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

	bool first_input = true;
	while (ros::ok() && dmmode != resetting && (trial_duration = ros::Time::now() - startt).toSec() < nominal_duration) {
		if (dmmode == running) {
			if (fresh_input) {
				mtx_.lock();
				fresh_input = false;

				if (scribethread)
					inputs_recording.enqueue( get_trial_number(), U );

				if (first_input) {
					first_input = false;
					if (U.size() == 0) { // Controller declares "unrealizable"
						mtx_.unlock();
						realizable = false;
						break;
					} else {
						realizable = true;
					}
					if (scribethread)
						times_recording.enqueue( get_trial_number(),
												 ros::Time::now() );
				}

				tg.step( probinstance->period, U );

				mtx_.unlock();
			} else {
				if (scribethread)
					inputs_recording.enqueue( get_trial_number(), defaultU );
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

	// Handle special cases of trial termination:
	if (ros::ok() && trial_duration.toSec() >= nominal_duration) {
		// The trial end because the nominal (hidden) duration was reached
		ROS_INFO( "dynamaestro: Trial ended after %f s (threshold is %d s).",
				  trial_duration.toSec(), nominal_duration );
	} else if (ros::ok() && !first_input && !realizable) {
		ROS_INFO( "dynamaestro: Controller declared \"unrealizable\" after %f s (trial duration threshold is %d s).",
				  trial_duration.toSec(), nominal_duration );
	}

	// Clear registered instance
	ROS_INFO( "dynamaestro: Clearing instance summary from Parameter Server." );
	nh_.setParam( "number_integrators", -1 );
	nh_.setParam( "output_dim", -1 );
	nh_.setParam( "period", -1.0 );
	fresh_input = false;
	labeler.clear();
	if (scribethread == NULL)
		delete probinstance;
	probinstance = NULL;
}

void tgthread( ros::NodeHandle &nhp )
{
	TGThread tgt( nhp );
	std::string filename;
	if (nhp.getParam( "results_file", filename ))
		tgt.start_monitoring( filename, true );
	int number_trials = 0;
	bool counting_trials = nhp.getParam( "/number_trials", number_trials );
	if (counting_trials)
		ROS_INFO( "dynamaestro: Initiated with request for %d trials.",
				  number_trials );
	int trial_counter = 0;
	while ((!counting_trials || trial_counter < number_trials)
		   && ros::ok()) {
		tgt.run();
		if (counting_trials)
			trial_counter++;
	}

	if (counting_trials && ros::ok()) {
		ROS_INFO( "dynamaestro: Completed %d trials.", trial_counter );
		tgt.stop_monitoring();
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

	boost::thread tgmain( tgthread, nh );
	tgmain.join();
	nh.shutdown();
	
#endif
	return 0;
}
