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
#include "integrator_chains_msgs/VectorStamped.h"
#include "integrator_chains_msgs/LabelStamped.h"
#include "integrator_chains_msgs/ProblemInstanceJSON.h"
#include "integrator_chains_msgs/DMMode.h"
#endif

#include "problem.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <list>
#include <cstdlib>
#include <cstdio>
#include <errno.h>
#include <cassert>

#include <Eigen/Dense>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


using namespace integrator_chains;


/** Flow computation for the problem domain: scaling chains of integrators.

   The trajectories are the solutions of a linear time-invariant control
   system. Control input is applied using a zero-order hold, i.e., applied
   constantly during the duration given as the first parameter of step().

   Let m be the highest order of derivative, and let n be the dimension of the
   output space.  The state variable indexing is such that the first output
   variable is the first state variable, the second output variable is the
   second state variable, etc. Thus the component systems are interleaved in
   the sense that the first subsystem is formed from state variable indices
   1, 1+n, 1+2n, ..., 1+(m-1)n, and the input to this subsystem is applied at
   state variable index 1+(m-1)n.

   \ingroup integrator_chains */
class TrajectoryGenerator {
public:
    TrajectoryGenerator( int numdim_output, int highest_order_deriv );

    /* Consult the class description regarding the assumed arrangement of the
       state variables, i.e., the indexing and the assumption that the system is
       a chain of integrators. */
    TrajectoryGenerator( Eigen::VectorXd Xinit, int numdim_output );

    void clear();

    /** Forward Euler integration for duration dt using constant input U */
    double step( double dt, const Eigen::VectorXd &U );

    double get_time() const
        { return t; }
    double get_state_dim() const
        { return X.size(); }
    double get_output_dim() const
        { return numdim_output; }

    /** Output vector */
    double operator[]( int i ) const;

    /** Access to full, current state vector */
    double get_state( int i ) const
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
    assert( U.size() > 0 );
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
    int peek_id();
    T peek();
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
int Queue<T>::peek_id()
{
    assert( this->size() > 0 );
    mtx_.lock();
    int id = Q.back().first;
    mtx_.unlock();
    return id;
}

template <typename T>
T Queue<T>::peek()
{
    assert( this->size() > 0 );
    mtx_.lock();
    T value = Q.back().second;
    mtx_.unlock();
    return value;
}


/** \ingroup integrator_chains */
class TGThread {
public:
    TGThread( ros::NodeHandle &nh );
    ~TGThread();
    void inputcb( const integrator_chains_msgs::VectorStamped &vs );
    bool mode_request( integrator_chains_msgs::DMMode::Request &req,
                       integrator_chains_msgs::DMMode::Response &res );
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
    std::pair<int,Problem *> random_known(const Eigen::Vector2i &numdim_output_bounds,
                           const Eigen::Vector2i &highest_order_deriv_bounds,
                           const Eigen::VectorXd &Y_box,
                           const Eigen::VectorXd &U_box,
                           const Eigen::Vector2i &number_goals_bounds,
                           const Eigen::Vector2i &number_obstacles_bounds,
                           const Eigen::Vector2d &period_bounds,
                           const Eigen::VectorXd &Xinit_bounds );

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
    Queue<std::pair<int,Problem *>> instances_recording;
    Queue<Eigen::VectorXd> inputs_recording;
    Queue<integrator_chains_msgs::VectorStamped *> states_recording;
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
    std::pair<int, std::pair<int,Problem *>> tp;//(-1, (-1, nullptr) );

    ros::Rate polling_rate( 1000 );
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

        if (tp.second.second)
	  delete tp.second.second;
        std::pair<int, std::pair<int,Problem *>> tp = instances_recording.dequeue();
        std::pair<int, ros::Time> tdurationt = times_recording.dequeue();
        assert( tdurationt.first == tp.first && tdurationt.second.nsec == 0 );

        // Wait for first state to be sure that it is safe to begin recording
        while (ros::ok() && get_trial_number() >= 0
               && states_recording.size() == 0
               && instances_recording.size() == 0)
            polling_rate.sleep();

        // Instance generated and queued but never revealed to the controller?
        if (states_recording.size() == 0 || states_recording.peek_id() != tp.first)
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
             << "  \"problem_instance\": " << *tp.second.second << "],\n"
             << "  \"is_realizable\": " << tp.second.first;

        // Wait for first input to get realizability declaration
        while (ros::ok() && get_trial_number() >= 0
               && inputs_recording.size() == 0
               && instances_recording.size() == 0) {
            polling_rate.sleep();
        }

        // Did the controller time-out, and the dm progress to the next trial?
        if (inputs_recording.size() == 0 || inputs_recording.peek_id() != tp.first) {
            outf << "}" << std::endl;
            continue;
        }

        // Drop initial state because it is already in the instance description
        states_recording.dequeue();

        bool at_least_one = false;
        while ((inputs_recording.size() > 0
                && inputs_recording.peek_id() == tp.first)
               || (ros::ok() && get_trial_number() >= 0
                   && !boost::this_thread::interruption_requested()
                   && instances_recording.size() == 0)) {

            if (inputs_recording.size() == 0
                || (states_recording.size() == 0 && inputs_recording.peek().size() > 0)) {
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

            std::pair<int, integrator_chains_msgs::VectorStamped *> tstate = states_recording.dequeue();

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
 
    if (tp.second.second)
      delete tp.second.second;
}

void TGThread::start_monitoring( std::string filename, bool append_mode )
{
    assert( scribethread == nullptr );
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
        scribethread = nullptr;
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

    char *prev_endptr = nullptr;
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
      probinstance(nullptr),
      labeler(),
      trial_number(-1),
      dmmode(paused),
      scribethread(nullptr)
{
    mode_srv = nh_.advertiseService( "mode", &TGThread::mode_request, this );
    problemJSONpub = nh_.advertise<integrator_chains_msgs::ProblemInstanceJSON>( "probleminstance_JSON", 1, true );
    statepub = nh_.advertise<integrator_chains_msgs::VectorStamped>( "state", 10, true );
    loutpub = nh_.advertise<integrator_chains_msgs::LabelStamped>( "loutput", 10, true );
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
    integrator_chains_msgs::VectorStamped *pt = new integrator_chains_msgs::VectorStamped();
    pt->header.frame_id = std::string( "map" );
    pt->header.stamp = ros::Time::now();
    for (int i = 0; i < tg.get_state_dim(); i++) {
        pt->v.point.push_back( tg.get_state( i ) );
        if (i < tg.get_output_dim())
            Y(i) = tg[i];
    }
    statepub.publish( *pt );

    std::list<std::string> label = labeler.get_label( Y );
    integrator_chains_msgs::LabelStamped lbl;
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

bool TGThread::mode_request( integrator_chains_msgs::DMMode::Request &req,
                             integrator_chains_msgs::DMMode::Response &res )
{
    switch (req.mode) {
    case integrator_chains_msgs::DMMode::Request::UNPAUSE:
        if (dmmode != running)
            dmmode = running;
        res.result = true;
        break;

    case integrator_chains_msgs::DMMode::Request::READY:
        if (dmmode == ready) {
            res.result = true;
        } else {
            res.result = false;
        }
        break;

    case integrator_chains_msgs::DMMode::Request::START:
        if (dmmode == ready) {
            dmmode = waiting;
            res.result = true;
        } else {
            res.result = false;
        }
        break;

    case integrator_chains_msgs::DMMode::Request::PAUSE:
        if (dmmode == running) {
            dmmode = paused;
            fresh_input = false;
        }
        res.result = true;
        break;

    case integrator_chains_msgs::DMMode::Request::RESTART:
        dmmode = restarting;
        ROS_INFO( "integrator_chains_msgs: Restarting trial..." );
        res.result = true;
        break;

    case integrator_chains_msgs::DMMode::Request::RESET:
        dmmode = resetting;
        ROS_INFO( "dynamaestro: Stopping current trial and generating a new instance..." );
        res.result = true;
        break;

    default:
        return false;
    }

    return true;
}

void TGThread::inputcb( const integrator_chains_msgs::VectorStamped &vs )
{
    assert( probinstance != nullptr );

    mtx_.lock();

    if (vs.v.point.size() == 0) {
        U.resize( 0 );  // Controller declares trial as not realizable.
    } else if (U.size() > 0) {
        for (int i = 0; i < probinstance->get_numdim_output(); i++)
            U[i] = vs.v.point[i];
    }
    fresh_input = true;

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
    if (duration_bounds(0) != duration_bounds(1))
        nominal_duration += std::rand() % (1+duration_bounds(1)-duration_bounds(0));

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

    std::pair<int, Problem *> prob =TGThread::random_known( numdim_output_bounds,
                                    num_integrators_bounds,
                                    Y_box, U_box,
                                    number_goals_bounds,
                                    number_obstacles_bounds,
                                    period_bounds,
                                    Y_box );
    probinstance = prob.second;
    if (probinstance->get_highest_order_deriv() > 1)
        probinstance->Xinit.tail( (probinstance->get_highest_order_deriv()-1)*probinstance->get_numdim_output() ).setZero();
    labeler.importProblem( *probinstance );

    U.setZero( probinstance->get_numdim_output() );
    Eigen::VectorXd Y( probinstance->get_numdim_output() );

    TrajectoryGenerator tg( probinstance->Xinit,
                            probinstance->get_numdim_output() );

    ros::Rate rate( 1/probinstance->period );
    Eigen::VectorXd defaultU;
    defaultU.setZero( U.size() );

    if (scribethread) {
        times_recording.enqueue( get_trial_number(),
                                 ros::Time( nominal_duration, 0 ) );
        instances_recording.enqueue( get_trial_number(), prob);
    }

    ros::Duration trial_duration;

    ros::Rate polling_rate( 100 );

    /* For this single spinOnce() to successfully clear any remaining input, it
       must be that the Subscriber to /input has queue length of 1. */
    ros::spinOnce();
    fresh_input = false;

    dmmode = ready;
    while (dmmode == ready) {
        if (!ros::ok())
            return;
        ros::spinOnce();
        polling_rate.sleep();
    }
    assert( dmmode == waiting );

    integrator_chains_msgs::ProblemInstanceJSON probinstance_msg;
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
                    if (scribethread)
                        times_recording.enqueue( get_trial_number(),
                                                 ros::Time::now() );
                    first_input = false;
                    if (U.size() == 0) { // Controller declares "unrealizable"
                        mtx_.unlock();
                        realizable = false;
                        break;
                    } else {
                        realizable = true;
                    }
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
    integrator_chains_msgs::VectorStamped pt;
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
    if (scribethread == nullptr)
        delete probinstance;
    probinstance = nullptr;
}


std::pair<int,Problem *>  TGThread::random_known( const Eigen::Vector2i &numdim_output_bounds,
                           const Eigen::Vector2i &highest_order_deriv_bounds,
                           const Eigen::VectorXd &Y_box,
                           const Eigen::VectorXd &U_box,
                           const Eigen::Vector2i &number_goals_bounds,
                           const Eigen::Vector2i &number_obstacles_bounds,
                           const Eigen::Vector2d &period_bounds,
                           const Eigen::VectorXd &Xinit_bounds )
{
    assert( numdim_output_bounds(0) >= 1
            && numdim_output_bounds(0) <= numdim_output_bounds(1)
            && highest_order_deriv_bounds(0) >= 1
            && highest_order_deriv_bounds(0) <= highest_order_deriv_bounds(1)
            && number_goals_bounds(0) >= 0
            && number_goals_bounds(0) <= number_goals_bounds(1)
            && number_obstacles_bounds(0) >= 0
            && number_obstacles_bounds(0) <= number_obstacles_bounds(1)
            && Y_box.size() >= 2*numdim_output_bounds(1)
            && U_box.size() >= 2*numdim_output_bounds(1)
            && period_bounds(0) >= 0 && period_bounds(1) >= period_bounds(0) );

    int i, j, k;
    Problem *prob = new Problem;
    int numdim_output, highest_order_deriv;
 
    prob->set_numdim_output(numdim_output_bounds(0));
    numdim_output = prob->get_numdim_output();
    if (numdim_output_bounds(1) != numdim_output_bounds(0))
      prob->set_numdim_output(numdim_output + (std::rand()
                                % (1+numdim_output_bounds(1)
                                   - numdim_output_bounds(0))));
    numdim_output - prob->get_numdim_output();

    prob->set_highest_order_deriv(highest_order_deriv_bounds(0));
    if (highest_order_deriv_bounds(1) != highest_order_deriv_bounds(0))
      highest_order_deriv = prob->get_highest_order_deriv();
      prob->set_highest_order_deriv(highest_order_deriv + (std::rand()
                                      % (1+highest_order_deriv_bounds(1)
                                         - highest_order_deriv_bounds(0))));
      highest_order_deriv = prob->get_highest_order_deriv();

    int number_goals = number_goals_bounds(0);
    if (number_goals_bounds(1) != number_goals_bounds(0))
        number_goals += (std::rand()
                         % (1+number_goals_bounds(1)
                            - number_goals_bounds(0)));

    int number_obstacles = number_obstacles_bounds(0);
    if (number_obstacles_bounds(1) != number_obstacles_bounds(0))
        number_obstacles += (std::rand()
                         % (1+number_obstacles_bounds(1)
                            - number_obstacles_bounds(0)));


    prob->Y = Polytope::box( Y_box.head( 2*numdim_output ) );
    prob->U = Polytope::box( U_box.head( 2*numdim_output ) );

    prob->Xinit.setZero( numdim_output*highest_order_deriv );
    int largest_dim = numdim_output*highest_order_deriv;
    if (largest_dim > Xinit_bounds.size()/2)
        largest_dim = Xinit_bounds.size()/2;
    for (i = 0; i < largest_dim; i++) {
        assert( Xinit_bounds(2*i+1) >= Xinit_bounds(2*i) );
        prob->Xinit(i) = Xinit_bounds(2*i);
        if (Xinit_bounds(2*i+1) != Xinit_bounds(2*i))
            prob->Xinit(i) += (double(std::rand())/RAND_MAX)*(Xinit_bounds(2*i+1) - Xinit_bounds(2*i));
    }

    Eigen::VectorXd box_bounds(2*numdim_output);
    prob->goals.resize( number_goals );
    for (i = 0; i < number_goals; i++) {
        for (j = 0; j < numdim_output; j++) {
            box_bounds(2*j) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
            box_bounds(2*j+1) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
            if (box_bounds(2*j+1) < box_bounds(2*j)) {
                double tmp = box_bounds(2*j+1);
                box_bounds(2*j+1) = box_bounds(2*j);
                box_bounds(2*j) = tmp;
            }
        }
        prob->goals[i] = LabeledPolytope::box( box_bounds,
                                               std::string("goal_") + Problem::get_int_to_str(i) );
    }

    prob->period = period_bounds(0);
    if (period_bounds(1) != period_bounds(0))
        prob->period += double(std::rand())/RAND_MAX*(period_bounds(1) - period_bounds(0));


    // randomly decide whether this problem is going to be realizable
    int realizable = (std::rand()%2);
    if (number_goals==1 && prob->goals[0]->is_in(prob->Xinit.topRows(numdim_output))) {
	    realizable = 1;
    }
    if (realizable==0) {

      /** generate obstacles accordingly, choose between the following 
          5 types of unrealizable specs:

          0. initial state contained in obstacle
          1. goal contained in obstacle
          2. union of obstacles covers some dimension
          3. initial state encased by 4 obstacles
          4. goal encased by 4 obstacles

	  So far only 0-2 are implemented. */

      int unreal_case;
      unreal_case = (std::rand() % 5);
      
      switch(unreal_case){

        case 0: {
	  prob->obstacles.resize( number_obstacles );
	  int encasing_obs = std::rand() % number_obstacles;
          for (j = 0; j < numdim_output; j++) {
	      box_bounds(2*j) = prob->Xinit(j)
		- (double(std::rand())/RAND_MAX)*(prob->Xinit(j) - Y_box(2*j));
	      box_bounds(2*j+1) = prob->Xinit(j)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - prob->Xinit(j)) ;
              if (box_bounds(2*j+1) < box_bounds(2*j)) {
                double tmp = box_bounds(2*j+1);
                box_bounds(2*j+1) = box_bounds(2*j);
                box_bounds(2*j) = tmp;
	      }

	    }
		  
	    prob->obstacles[encasing_obs] = LabeledPolytope::box( box_bounds,
							 std::string("obstacle_") + Problem::get_int_to_str(encasing_obs) );
	
	  for (i = 0; i < number_obstacles; i++) {
	    if (i==encasing_obs) {
	      continue;
	    }
	    for (j = 0; j < numdim_output; j++) {
	      box_bounds(2*j) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      box_bounds(2*j+1) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      if (box_bounds(2*j+1) < box_bounds(2*j)) {
                double tmp = box_bounds(2*j+1);
                box_bounds(2*j+1) = box_bounds(2*j);
                box_bounds(2*j) = tmp;
	      }
	    }
	    prob->obstacles[i] = LabeledPolytope::box( box_bounds,
						       std::string("obstacle_") + Problem::get_int_to_str(i) );

	  }
        }
      	
        case 1: {

	  Eigen::VectorXd last_goal_bounds = box_bounds;

	  prob->obstacles.resize( number_obstacles );

          int encasing_obs = std::rand() % number_obstacles;
          for (j = 0; j < numdim_output; j++) {
	      box_bounds(2*j) = last_goal_bounds(2*j)
		- (double(std::rand())/RAND_MAX)*(last_goal_bounds(2*j) - Y_box(2*j));
	      box_bounds(2*j+1) = last_goal_bounds(2*j+1)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - last_goal_bounds(2*j+1)) ;
	      if (box_bounds(2*j+1) < box_bounds(2*j)) {
		double tmp = box_bounds(2*j+1);
		box_bounds(2*j+1) = box_bounds(2*j);
		box_bounds(2*j) = tmp;
	      }	
	    }
		  
	    prob->obstacles[encasing_obs] = LabeledPolytope::box( box_bounds,
							 std::string("obstacle_") + Problem::get_int_to_str(encasing_obs) );
	
	  for (i = 0; i < number_obstacles; i++) {
	    if (i==encasing_obs) {
	      continue;
	    }
	    for (j = 0; j < numdim_output; j++) {
	      box_bounds(2*j) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      box_bounds(2*j+1) = Y_box(2*j)
                + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      if (box_bounds(2*j+1) < box_bounds(2*j)) {
                double tmp = box_bounds(2*j+1);
                box_bounds(2*j+1) = box_bounds(2*j);
                box_bounds(2*j) = tmp;
	      }
	    }
	    prob->obstacles[i] = LabeledPolytope::box( box_bounds,
						       std::string("obstacle_") + Problem::get_int_to_str(i) );

	  }
        }

        case 2: {
	  prob->obstacles.resize( number_obstacles );

          int cut_obs = std::rand() % number_obstacles;
       
          int cut_dim1 = std::rand() % numdim_output;
          int cut_dim2 = std::rand() % numdim_output;

	  while (numdim_output!=1 && cut_dim1==cut_dim2) {
            cut_dim2 = std::rand() % numdim_output;
	  }
          int cut_goal = std::rand() % number_goals;
          Eigen::VectorXd goal_bounds = prob->goals[cut_goal]->get_bounds();
	  int split = (prob->Xinit(cut_dim1)+goal_bounds(2*cut_dim1))/2;
	  box_bounds(2*cut_dim1) = split 
	    - (double(std::rand())/RAND_MAX) *std::abs(split-prob->Xinit(cut_dim1));
	  box_bounds(2*cut_dim1+1) = split 
	    + (double(std::rand())/RAND_MAX)*std::abs(split-prob->Xinit(cut_dim1));

	  box_bounds(2*cut_dim2) = Y_box(2*cut_dim2);
	  box_bounds(2*cut_dim2+1) = Y_box(2*cut_dim2+1);

	  for (j = 0; j < numdim_output; j++) {
	    if (j==cut_dim1 || j==cut_dim2) {
	      continue;
	    }
	    box_bounds(2*j) = Y_box(2*j)
	      + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	    box_bounds(2*j+1) = Y_box(2*j)
	      + (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	    if (box_bounds(2*j+1) < box_bounds(2*j)) {
	      double tmp = box_bounds(2*j+1);
	      box_bounds(2*j+1) = box_bounds(2*j);
	      box_bounds(2*j) = tmp;
	    }
	  }
	    
	  
	  prob->obstacles[cut_obs] = LabeledPolytope::box( box_bounds,
							   std::string("obstacle_") + Problem::get_int_to_str(cut_obs) );
	
	  for (i = 0; i < number_obstacles; i++) {
	    if (i==cut_obs) {
	      continue;
	    }
	    for (j = 0; j < numdim_output; j++) {
	      box_bounds(2*j) = Y_box(2*j)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      box_bounds(2*j+1) = Y_box(2*j)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      if (box_bounds(2*j+1) < box_bounds(2*j)) {
		double tmp = box_bounds(2*j+1);
		box_bounds(2*j+1) = box_bounds(2*j);
		box_bounds(2*j) = tmp;
	      }
	    }
	    prob->obstacles[i] = LabeledPolytope::box( box_bounds,
						       std::string("obstacle_") + Problem::get_int_to_str(i) );
	    
	  }
	}

        case 3: {
        }

        case 4: {
        }

      }
   
    } else {

      // generate obstacles that do not overlap with one another
      // in any dimension

      bool overlap,init_in_obs;
      int max_min, max_max, min_min, min_max;
      Eigen::VectorXd prev_bounds;
      prob->obstacles.resize( number_obstacles );
      for (i = 0; i < number_obstacles; i++) {
        init_in_obs=true;
        while (init_in_obs) {
	  for (j = 0; j < numdim_output; j++) {
	    overlap = true;
	    while (overlap) {
	      box_bounds(2*j) = Y_box(2*j)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      box_bounds(2*j+1) = Y_box(2*j)
		+ (double(std::rand())/RAND_MAX)*(Y_box(2*j+1) - Y_box(2*j));
	      if (box_bounds(2*j+1) < box_bounds(2*j)) {
		double tmp = box_bounds(2*j+1);
		box_bounds(2*j+1) = box_bounds(2*j);
		box_bounds(2*j) = tmp;
	      }	
	      for (k = 0; k < i; k++) {
                prev_bounds = prob->obstacles[k]->get_bounds();
		max_min = std::max(box_bounds(2*j),prev_bounds(2*j));
		min_max = std::min(box_bounds(2*j+1),prev_bounds(2*j+1));
		if (max_min <= min_max) {
		  break;
		}
	      }
	      if (k==i) {
		overlap = false;
	      }
	    }
	  }

	  prob->obstacles[i] = LabeledPolytope::box( box_bounds,
						     std::string("obstacle_") + Problem::get_int_to_str(i) );

          if (!prob->obstacles[i]->is_in((prob->Xinit.topRows(numdim_output)))) init_in_obs=false; 

	}
      }
    }
    return std::pair<int,Problem*>(realizable,prob);
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

    time_t seed = time( nullptr );
    std::srand( seed );
    ROS_INFO( "dynamaestro: Using %ld as the PRNG seed.", seed );

    boost::thread tgmain( tgthread, nh );
    tgmain.join();
    nh.shutdown();

#endif
    return 0;
}
