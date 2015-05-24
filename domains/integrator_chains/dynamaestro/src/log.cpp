/* log.cpp - logging and transcription of messages to standard formats.
 *
 * SCL; 2015
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "dynamaestro/VectorStamped.h"
#include "dynamaestro/LabelStamped.h"

#include <unistd.h>
#include <assert.h>


/** \ingroup integrator_chains */
class DMTranscriber {
public:
	DMTranscriber( ros::NodeHandle &nh, int i0, int i1=-1, int i2=-1 );
	void statecb( const dynamaestro::VectorStamped &vs );

private:
	ros::NodeHandle &nh_;
	ros::Publisher pubPointStamped;
	ros::Subscriber subVectorStamped;

	int mapped_i0, mapped_i1, mapped_i2;
};

/** Instantiate transcriber with given mapping.

  \param i0 index of the vector that should be mapped to Point::x

  \param i1 index of the vector that should be mapped to Point::y, or -1 if no
  mapping should occur for y, in which case Point::y is assigned 0.

  \param i2 analogous to \p i1 but for Point::z.
 */
DMTranscriber::DMTranscriber( ros::NodeHandle &nh, int i0, int i1, int i2 )
	: nh_(nh), mapped_i0(i0), mapped_i1(i1), mapped_i2(i2)
{
	assert( i0 >= 0 && i1 >= -1 && i2 >= -1 );

	ROS_INFO( "dynamaestro_logger: Using the mapping (%d,%d,%d) -> (x,%s,%s)",
			  mapped_i0, mapped_i1, mapped_i2,
			  mapped_i1 >= 0 ? "y" : "0",
			  mapped_i2 >= 0 ? "z" : "0" );

	pubPointStamped = nh_.advertise<geometry_msgs::PointStamped>( "state_PointStamped", 10, true );
	subVectorStamped = nh_.subscribe( "/dynamaestro/state", 1, &DMTranscriber::statecb, this );
}

void DMTranscriber::statecb( const dynamaestro::VectorStamped &vs )
{
	geometry_msgs::PointStamped pt;
	pt.header.frame_id = vs.header.frame_id;
	pt.header.stamp = vs.header.stamp;
	pt.point.x = vs.v.point[mapped_i0];
	if (mapped_i1 >= 0) {
		pt.point.y = vs.v.point[mapped_i1];
	} else {
		pt.point.y = 0.0;
	}
	if (mapped_i2 >= 0) {
		pt.point.z = vs.v.point[mapped_i2];
	} else {
		pt.point.z = 0.0;
	}
	pubPointStamped.publish( pt );
}


class WordEvents {
public:
	WordEvents( ros::NodeHandle &nh );
	void labelcb( const dynamaestro::LabelStamped &ls );

private:
	ros::NodeHandle &nh_;
	ros::Publisher pubLabelingNoRep;
	ros::Subscriber subLabeledOutput;
	std::vector<std::string> prevlabel;
	bool initialized;
};

WordEvents::WordEvents( ros::NodeHandle &nh )
	: nh_(nh), initialized(false)
{
	pubLabelingNoRep = nh_.advertise<dynamaestro::LabelStamped>( "loutput_norep", 10, true );
	subLabeledOutput = nh_.subscribe( "/dynamaestro/loutput", 10, &WordEvents::labelcb, this );
}

void WordEvents::labelcb( const dynamaestro::LabelStamped &ls )
{
	if (!initialized || ls.label != prevlabel) {
		initialized = true;
		prevlabel = ls.label;
		dynamaestro::LabelStamped echoed_ls = ls;
		pubLabelingNoRep.publish( echoed_ls );
	}
}


int main( int argc, char **argv )
{
	ros::init( argc, argv, "dynamaestro_logger" );
	ros::NodeHandle nh( "~" );

	int numdim_output;
	while (!nh.getParam( "/dynamaestro/output_dim", numdim_output )) {
		ROS_INFO( "dynamaestro_logger: number_integrators parameter not present; will try again soon..." );
		sleep( 1 );
	}

	int i0 = 0, i1 = -1, i2 = -1;
	if (numdim_output > 1)
		i1 = 1;
	if (numdim_output > 2)
		i2 = 2;

	DMTranscriber dmt( nh, i0, i1, i2 );
	WordEvents we( nh );
	tf::TransformListener tflistener( nh );
	ros::spin();

	return 0;
}
