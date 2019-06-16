/**
 * @file /include/star_gui/main_window.hpp
 *
 * @brief Qt based gui for star_gui.
 *
 * @date November 2010
 **/
#ifndef star_gui_MAIN_WINDOW_H
#define star_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace star_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	// added for the star_gui
	void on_button_accept_stitch_clicked(bool check );
	void on_button_repeat_stitch_clicked(bool check );
	void on_button_send_points_clicked(bool check );
	void on_button_half_drive_clicked(bool check );
	void on_button_full_drive_clicked(bool check );

	void on_x_offset_valueChanged(int val);
	void on_y_offset_valueChanged(int val);
	
	void updateForce();
	void updateImage();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace star_gui

#endif // star_gui_MAIN_WINDOW_H
