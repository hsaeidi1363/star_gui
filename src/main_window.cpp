/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QPixmap>
#include <iostream>
#include "../include/star_gui/main_window.hpp"
#include <stdlib.h>
#include <ros/package.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace star_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    QPixmap pix((ros::package::getPath("star_gui") + "/resources/images/static_cam.png").c_str());
    ui.image_view->setPixmap(pix.scaled(200,160,Qt::KeepAspectRatio));

    QObject::connect(ui.slider_x_offset, SIGNAL(valueChanged(int)), SLOT(on_slider_x_offset_valueChanged(int)));
    QObject::connect(ui.slider_y_offset, SIGNAL(valueChanged(int)), SLOT(on_slider_y_offset_valueChanged(int)));
    QObject::connect(ui.slider_z_offset, SIGNAL(valueChanged(int)), SLOT(on_slider_z_offset_valueChanged(int)));

    QObject::connect(ui.dsp_x_offset, SIGNAL(valueChanged(double)), SLOT(on_dsp_x_offset_valueChanged(double)));
    QObject::connect(ui.dsp_y_offset, SIGNAL(valueChanged(double)), SLOT(on_dsp_y_offset_valueChanged(double)));
    QObject::connect(ui.dsp_z_offset, SIGNAL(valueChanged(double)), SLOT(on_dsp_z_offset_valueChanged(double)));

    if (qnode.global_offset){
	    ui.global_offseting->setCheckState(Checked);
    }else{
	    ui.single_point_offseting->setCheckState(Checked);
    }
    QObject::connect(&qnode, SIGNAL(forceUpdated()), this, SLOT(updateForce()));
    QObject::connect(&qnode, SIGNAL(imageUpdated()), this, SLOT(updateImage()));


    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    QObject::connect(&qnode, SIGNAL(forceUpdated()), this, SLOT(updateForce()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}




void MainWindow::on_button_accept_stitch_clicked(bool check ) {
	qnode.accept_stitch = true;
	
}

void MainWindow::on_button_accept_tension_clicked(bool check ) {
	qnode.accept_tension = true;
	
}

void MainWindow::on_button_repeat_stitch_clicked(bool check ) {
	qnode.repeat_stitch = true;
	
}

void MainWindow::on_button_accept_offset_clicked(bool check ) {
	qnode.accept_offset = true;
	
}

void MainWindow::on_button_send_points_clicked(bool check ) {
	qnode.send_points = true;
	
}


void MainWindow::on_button_save_rcm_clicked(bool check ) {
	qnode.save_rcm = true;
	
}

void MainWindow::on_button_set_rcm_clicked(bool check ) {
	qnode.set_rcm = true;
	
}


void MainWindow::on_button_half_drive_clicked(bool check ) {
	//system((ros::package::getPath("endo360_rtt") + "/scripts/endo360-halfdrv.sh").c_str());
        system((ros::package::getPath("proxisure_rtt") + "/scripts/proxisure-halfdrv.sh").c_str());
        qnode.half_drive = true;
	
}

void MainWindow::on_button_full_drive_clicked(bool check ) {
	//system((ros::package::getPath("endo360_rtt") + "/scripts/endo360-fulldrv.sh").c_str());
	system((ros::package::getPath("proxisure_rtt") + "/scripts/proxisure-fulldrv.sh").c_str());
        qnode.full_drive = true;
}



void MainWindow::on_button_reset_x_offset_clicked(bool check ){
	if(qnode.global_offset){
		qnode.x_offset_global = 0.0;

	}else{
		qnode.x_offset_single = 0.0;
	}
	ui.slider_x_offset->setValue(0);
	ui.dsp_x_offset->setValue(0.0);

}	

void MainWindow::on_button_reset_y_offset_clicked(bool check ){
	if(qnode.global_offset){
		qnode.y_offset_global = 0.0;

	}else{
		qnode.y_offset_single = 0.0;
	}
	ui.slider_y_offset->setValue(0);
	ui.dsp_y_offset->setValue(0.0);

}	

void MainWindow::on_button_reset_z_offset_clicked(bool check ){
	if(qnode.global_offset){
		qnode.z_offset_global = 0.0;

	}else{
		qnode.z_offset_single = 0.0;
	}
	ui.slider_z_offset->setValue(0);
	ui.dsp_z_offset->setValue(0.0);

}	


void MainWindow::on_global_offseting_stateChanged(int state ) {
	qnode.global_offset = true;
	ui.single_point_offseting->setCheckState(Unchecked);
}

void MainWindow::on_single_point_offseting_stateChanged(int state ) {
	qnode.global_offset = false;
	ui.global_offseting->setCheckState(Unchecked);
	
}



void MainWindow::on_slider_x_offset_valueChanged(int val){
	if(qnode.global_offset){
		qnode.x_offset_global = val*0.05;
		ui.dsp_x_offset->setValue(qnode.x_offset_global);
	}else{
		qnode.x_offset_single = val*0.05;
		ui.dsp_x_offset->setValue(qnode.x_offset_single);	
	}
}


void MainWindow::on_slider_y_offset_valueChanged(int val){
	if(qnode.global_offset){
		qnode.y_offset_global = val*0.05;
		ui.dsp_y_offset->setValue(qnode.y_offset_global);
	}else{
		qnode.y_offset_single = val*0.05;
		ui.dsp_y_offset->setValue(qnode.y_offset_single);	
	}
}


void MainWindow::on_slider_z_offset_valueChanged(int val){
	if(qnode.global_offset){	
		qnode.z_offset_global = val*0.05;
		ui.dsp_z_offset->setValue(qnode.z_offset_global);
	}else{
		qnode.z_offset_single = val*0.05;
		ui.dsp_z_offset->setValue(qnode.z_offset_single);	
	}
	
}






void MainWindow::on_dsp_x_offset_valueChanged(double val){
	if(qnode.global_offset){
		qnode.x_offset_global = val;
	}else{
		qnode.x_offset_single = val;
	}
	ui.slider_x_offset->setValue((int) (val*20));
}

void MainWindow::on_dsp_y_offset_valueChanged(double val){
	if(qnode.global_offset){
		qnode.y_offset_global = val;
	}else{
		qnode.y_offset_single = val;
	}
	ui.slider_y_offset->setValue((int) (val*20));
}

void MainWindow::on_dsp_z_offset_valueChanged(double val){
	if(qnode.global_offset){
		qnode.z_offset_global = val;
	}else{
		qnode.z_offset_single = val;
	}
	ui.slider_z_offset->setValue((int) (val*20));
}



void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateForce() {
        ui.force_x->setValue(qnode.force_val.force.x);
        ui.force_y->setValue(qnode.force_val.force.y);
        ui.force_z->setValue(qnode.force_val.force.z);
}


void MainWindow::updateImage() {
        int width;
	int height;
	width = (int) qnode.cam_img.width;
	height = (int) qnode.cam_img.height;
	cv::Mat img;
	cv_bridge::CvImagePtr cv_ptr;

//        cv_ptr = cv_bridge::toCvCopy(qnode.cam_img,"mono8");
        cv_ptr = cv_bridge::toCvCopy(qnode.cam_img,"rgb8");
	// take the image part and put it in a mat variable in opencv format
	img = cv_ptr->image;
	
   
//    	QImage qim1((const uchar *) img.data, img.cols, img.rows, img.step, QImage::Format_Mono);
    	QImage qim1((const uchar *) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);

    	
	QPixmap pix = QPixmap::fromImage(qim1);
     	
    	int width_view = (int) width/5;
	int height_view = (int) height/5;
  	ui.image_view->setPixmap(pix.scaled(width_view,height_view,Qt::KeepAspectRatio));
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "star_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "star_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace star_gui

