/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include "../include/star_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace star_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}


void QNode::getForce(const geometry_msgs::WrenchStamped & _data){
	
	//std_msgs::String msg;
	//std::stringstream ss;
	//ss <<_data.data;
	//msg.data = ss.str();
	this->force_val = _data.wrench;
        //ROS_INFO("force x is: %f",this->force_val.force.x);
//	log(Info,std::string("I received a force reading"));
	Q_EMIT forceUpdated(); 

}

void QNode::getImage(const sensor_msgs::Image & _data){
	
	
	this->cam_img = _data;
   //     ROS_INFO("recieved an image of widht: %d and height %d",this->cam_img.width, this->cam_img.height);
//	log(Info,std::string("I received a force reading"));
	Q_EMIT imageUpdated(); 

}
bool QNode::init() {
	ros::init(init_argc,init_argv,"star_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	x_offset_global = 0.0;
	y_offset_global = 0.0;
	z_offset_global = 0.0;
	x_offset_single = 0.0;
	y_offset_single = 0.0;
	z_offset_single = 0.0;
	accept_stitch = false;
	repeat_stitch = false;
	send_points = false;
	half_drive = false;
	full_drive = false;
	global_offset = true;

	offset_publisher = n.advertise<geometry_msgs::Twist>("stitch_offset", 10);
	accept_stitch_publisher = n.advertise<std_msgs::Bool>("/suture/accept_stitch", 10);
	repeat_stitch_publisher = n.advertise<std_msgs::Bool>("/suture/repeat_stitch", 10);
	send_points_publisher = n.advertise<std_msgs::Bool>("send_plan", 10);
	half_drive_publisher = n.advertise<std_msgs::Bool>("half_drive", 10);
	full_drive_publisher = n.advertise<std_msgs::Bool>("full_drive", 10);

	force_subscriber = n.subscribe("ft20610", 1, &QNode::getForce, this);
	image_subscriber = n.subscribe("/see_scope/nir/image_raw", 1, &QNode::getImage, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"star_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	x_offset_global = 0.0;
	y_offset_global = 0.0;
	z_offset_global = 0.0;
	x_offset_single = 0.0;
	y_offset_single = 0.0;
	z_offset_single = 0.0;
	accept_stitch = false;
	repeat_stitch = false;
	send_points = false;
	half_drive = false;
	full_drive = false;
	global_offset = true;

	offset_publisher = n.advertise<geometry_msgs::Twist>("stitch_offset", 10);
	accept_stitch_publisher = n.advertise<std_msgs::Bool>("/suture/accept_stitch", 10);
	repeat_stitch_publisher = n.advertise<std_msgs::Bool>("/suture/repeat_stitch", 10);
	send_points_publisher = n.advertise<std_msgs::Bool>("send_plan", 10);
	half_drive_publisher = n.advertise<std_msgs::Bool>("half_drive", 10);
	full_drive_publisher = n.advertise<std_msgs::Bool>("full_drive", 10);

	force_subscriber = n.subscribe("ft20610", 1, &QNode::getForce, this);
	image_subscriber = n.subscribe("/see_scope/nir/image_raw", 1, &QNode::getImage, this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(20);
	int count = 0;
	while ( ros::ok() ) {

		if(accept_stitch){
			std_msgs::Bool accept_msg;
			accept_msg.data = true;
			accept_stitch_publisher.publish(accept_msg);
			accept_stitch = false;
		}


		if(repeat_stitch){
			std_msgs::Bool repeat_msg;
			repeat_msg.data = true;
			repeat_stitch_publisher.publish(repeat_msg);
			repeat_stitch = false;
		}

		if(send_points){
			std_msgs::Bool send_pts_msg;
			send_pts_msg.data = true;
			send_points_publisher.publish(send_pts_msg);
			send_points = false;
		}

		if(half_drive){
			std_msgs::Bool half_drive_msg;
			half_drive_msg.data = true;
			half_drive_publisher.publish(half_drive_msg);
			half_drive = false;
		}

		if(full_drive){
			std_msgs::Bool full_drive_msg;
			full_drive_msg.data = true;
			full_drive_publisher.publish(full_drive_msg);
			full_drive = false;
		}


		geometry_msgs::Twist offset_msg;
		offset_msg.linear.x = x_offset_global;
		offset_msg.linear.y = y_offset_global;
		offset_msg.linear.z = z_offset_global;
		offset_msg.angular.x = x_offset_single;
		offset_msg.angular.y = y_offset_single;		
		offset_msg.angular.z = z_offset_single;

		offset_publisher.publish(offset_msg);

	

		ros::spinOnce();
		loop_rate.sleep();
		++count;		

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace star_gui
