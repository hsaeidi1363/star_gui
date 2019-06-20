/**
 * @file /include/star_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef star_gui_QNODE_HPP_
#define star_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Image.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace star_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void getForce(const geometry_msgs::WrenchStamped & _data);
	void getImage(const sensor_msgs::Image & _data);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	geometry_msgs::Wrench force_val;
	sensor_msgs::Image cam_img;

	bool accept_stitch;
	bool accept_tension;
	bool accept_offset;
	bool repeat_stitch;
	bool send_points;
	bool half_drive;
	bool full_drive;
	bool global_offset;
	bool save_rcm;
	bool set_rcm;
	

	float x_offset_global;
	float y_offset_global;
	float z_offset_global;
	float x_offset_single;
	float y_offset_single;
	float z_offset_single;


Q_SIGNALS:
	void loggingUpdated();
   	void rosShutdown();
	void forceUpdated();
	void imageUpdated();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher offset_publisher;
	ros::Publisher accept_stitch_publisher;
	ros::Publisher accept_tension_publisher;
	ros::Publisher accept_offset_publisher;
	ros::Publisher repeat_stitch_publisher;
	ros::Publisher send_points_publisher;
	ros::Publisher half_drive_publisher;
	ros::Publisher full_drive_publisher;
	ros::Publisher save_rcm_publisher;
	ros::Publisher set_rcm_publisher;

	ros::Subscriber force_subscriber;
	ros::Subscriber image_subscriber;
	QStringListModel logging_model;
};

}  // namespace star_gui

#endif /* star_gui_QNODE_HPP_ */
