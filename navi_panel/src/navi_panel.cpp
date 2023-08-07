#include "navi_panel.hpp"

#include <QFile>
#include <QDir>
#include <QTimer>
#include <QFileDialog>
#include <QUiLoader>
#include <QVBoxLayout>
#include <ros/package.h>
#include <string.h>
#include <spot_msgs/ListGraph.h>
#include <spot_msgs/GraphRecording.h>
#include "ros/ros.h"


namespace navi_panel

{

    ControlPanel::ControlPanel(QWidget *parent) {
        std::string packagePath = ros::package::getPath("navi_panel") + "/resource/navigation_control.ui";
        ROS_INFO("Getting ui file from package path %s", packagePath.c_str());
        QFile file(packagePath.c_str());
        file.open(QIODevice::ReadOnly);

        QUiLoader loader;
        QWidget* ui = loader.load(&file, parent);
        file.close();

        QVBoxLayout* topLayout = new QVBoxLayout();
        this->setLayout(topLayout);
        topLayout->addWidget(ui);

        // member variables
        isRecording = false;
        recTimer = new QTimer(this);
        recElapsedTime = QTime(0, 0);

        // Setup ROS service clients
        startRecordingService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/start_recording");
        stopRecordingService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/stop_recording");
        getRecordingStatusService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/get_recording_status");
        downloadGraphService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/download_graph");
        uploadGraphService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/upload_graph");

        // Get Qt Widget handles
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        waypointToggleButton = this->findChild<QPushButton*>("waypointToggleButton");
        pointcloudToggleButton = this->findChild<QPushButton*>("pointcloudToggleButton");
        waypointNavButton = this->findChild<QPushButton*>("waypointNavButton");
        graphLoadButton = this->findChild<QPushButton*>("graphLoadButton");
        graphSaveButton = this->findChild<QPushButton*>("graphSaveButton");

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");

        statusBox = this->findChild<QTextEdit*>("statusBox");

        // Call Widget setup functions
        setupRecordingPanel();
        setupToggleButtons();
        setupModalButtons();
        setupStatusBox();

        // Setup ROS topic subscribers now that everything is set up
        // TODO: give these callback functions to subscribe to
        // graphWaypointsSub_ = nh_.subscribe("/spot/graph_waypoints", 1, &ControlPanel::waypointToggle, this);
        // graphEdgesSub_ = nh_.subscribe("/spot/graph_edges", 1, &ControlPanel::waypointToggle, this);

        // Connect Qt Signals and Slots
        connect(recordingToggleButton, SIGNAL(clicked()), this, SLOT(recordingToggle()));
        connect(recTimer, SIGNAL(timeout()), this, SLOT(tick()));
        connect(waypointToggleButton, SIGNAL(clicked()), this, SLOT(waypointToggle()));
        connect(pointcloudToggleButton, SIGNAL(clicked()), this, SLOT(pointcloudToggle()));
        connect(waypointNavButton, SIGNAL(clicked()), this, SLOT(waypointNav()));
        connect(graphLoadButton, SIGNAL(clicked()), this, SLOT(graphLoad()));
        connect(graphSaveButton, SIGNAL(clicked()), this, SLOT(graphSave()));
    }

    // Setup functions
    void ControlPanel::setupRecordingPanel() {
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        recordingToggleButton->setText(QString::fromUtf8("Start Recording"));
        QPalette pal = recordingToggleButton->palette();
        pal.setColor(QPalette::Button, QColor(255, 85, 90));
        recordingToggleButton->setAutoFillBackground(true);
        recordingToggleButton->setPalette(pal);
        recordingToggleButton->update();

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");
        recordingTime->setDigitCount(5);
        recordingTime->display(QString::fromUtf8("00:00"));
        recordingTime->update();
    }

    void ControlPanel::setupToggleButtons() {
        waypointToggleButton = this->findChild<QPushButton*>("waypointToggleButton");
        waypointToggleButton->setCheckable(true);
        waypointToggleButton->setChecked(false);
        waypointToggleButton->setText(QString::fromUtf8("Show Wapoints"));
        waypointToggleButton->update();

        pointcloudToggleButton = this->findChild<QPushButton*>("pointcloudToggleButton");
        pointcloudToggleButton->setCheckable(true);
        pointcloudToggleButton->setChecked(false);
        pointcloudToggleButton->setText(QString::fromUtf8("Show Point Cloud"));
        pointcloudToggleButton->update();
    }

    void ControlPanel::setupModalButtons() {
        graphLoadButton = this->findChild<QPushButton*>("graphLoadButton");
        graphLoadButton->setText(QString::fromUtf8("Load Graph..."));
        graphLoadButton->update();

        graphSaveButton = this->findChild<QPushButton*>("graphSaveButton");
        graphSaveButton->setText(QString::fromUtf8("Save Graph..."));
        graphSaveButton->update();

        waypointNavButton = this->findChild<QPushButton*>("waypointNavButton");
        QPalette pal = waypointNavButton->palette();
        pal.setColor(QPalette::Button, QColor(85, 90, 255));
        waypointNavButton->setAutoFillBackground(true);
        waypointNavButton->setPalette(pal);
        waypointNavButton->update();
    }

    void ControlPanel::setupStatusBox() {
        statusBox = this->findChild<QTextEdit*>("statusBox");
        statusBox->setReadOnly(true);
        statusBox->setPlainText(QString::fromUtf8("Log messages will appear here."));
        statusBox->update();
    }

    // Custom functions
    void ControlPanel::logStatus(QString msg) {
        statusBox->append(msg);
    }

    // ROS message callbacks
    bool ControlPanel::callGraphRecordingServices(ros::ServiceClient service, std::string serviceName, spot_msgs::GraphRecording serviceRequest) {
        std::string labelText = "Calling " + serviceName + " service";
        statusBox->setText(QString(labelText.c_str()));
        if (service.call(serviceRequest)) {
            if (serviceRequest.response.success) {
                labelText = "Successfully called " + serviceName + " service";
                statusBox->setText(QString(labelText.c_str()));
                return true;
            } else {
                labelText = serviceName + " service failed: " + serviceRequest.response.message;
                statusBox->setText(QString(labelText.c_str()));
                return false;
            }
        } else {
            labelText = "Failed to call " + serviceName + " service" + serviceRequest.response.message;
            statusBox->setText(QString(labelText.c_str()));
            return false;
        }
    }

    // Qt slot functions
    void ControlPanel::recordingToggle() {
        QPalette pal = recordingToggleButton->palette();

        if (isRecording) {
            logStatus(QString::fromUtf8("Stopping recording..."));
            pal.setColor(QPalette::Button, QColor(85, 255, 90));
            recordingToggleButton->setText(QString::fromUtf8("Start Recording"));

            // stop & reset recording timer
            recTimer->stop();
            recElapsedTime.setHMS(0, 0, 0);
            recordingTime->display(recElapsedTime.toString("mm:ss"));

            spot_msgs::GraphRecording req;
            req.request.path = std::string("");
            callGraphRecordingServices(stopRecordingService_, "stop recording", req);

            isRecording = false;
        } else {
            logStatus(QString::fromUtf8("Starting recording..."));
            pal.setColor(QPalette::Button, QColor(255, 85, 90));
            recordingToggleButton->setText(QString::fromUtf8("Stop Recording"));

            // start recording timer
            recTimer->start(1000);


            spot_msgs::GraphRecording req;
            req.request.path = std::string("");
            callGraphRecordingServices(startRecordingService_, "start recording", req);

            isRecording = true;
        }

        recordingToggleButton->setPalette(pal);
        recordingToggleButton->repaint();

        return;
    }

    void ControlPanel::tick() {
        QTime newTime = recElapsedTime.addSecs(1);
        QString text = newTime.toString("mm:ss");
        if (newTime.second() % 2 == 0) text[2] = ' ';
        recordingTime->display(text);
        recElapsedTime = newTime;
    }

    void ControlPanel::waypointToggle() {
        logStatus(QString::fromUtf8("Toggling waypoint visibility..."));

        // TODO toggle waypoint display

        return;
    }

    void ControlPanel::pointcloudToggle() {
        logStatus(QString::fromUtf8("Toggling pointcloud visibility..."));

        // TODO toggle pointcloud display

        return;
    }

    void ControlPanel::waypointNav() {
        logStatus(QString::fromUtf8("Opening navigation modal..."));

        // TODO disable button unless nav_markers is running

        // TODO navigate to selected waypoint

        return;
    }

    void ControlPanel::graphLoad() {
        logStatus(QString::fromUtf8("Opening file browser..."));

        QString dir = QFileDialog::getExistingDirectory(this, "Load Graph from Directory", QDir::currentPath(), QFileDialog::ShowDirsOnly | QFileDialog::ReadOnly);
        logStatus(dir);

        spot_msgs::GraphRecording req;
        req.request.path = dir.toStdString();
        callGraphRecordingServices(uploadGraphService_, "start recording", req);

        return;
    }

    void ControlPanel::graphSave() {
        QFileDialog dialog(this);
        dialog.setWindowTitle("Save Graph to Directory");
        dialog.setDirectory(QDir::currentPath());
        dialog.setFileMode(QFileDialog::Directory);
        dialog.setOption(QFileDialog::ShowDirsOnly, true);
        dialog.setAcceptMode(QFileDialog::AcceptSave);
        dialog.setOption(QFileDialog::DontUseNativeDialog, true);

        logStatus(QString::fromUtf8("Opening file browser..."));
        dialog.exec();
        QString dirPath = dialog.selectedFiles()[0];
        logStatus(dirPath);

        spot_msgs::GraphRecording req;
        req.request.path = dirPath.toStdString();
        callGraphRecordingServices(uploadGraphService_, "start recording", req);    

        return;
    }

    // Config file functions
    void ControlPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void ControlPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }

} // end namespace navi_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navi_panel::ControlPanel, rviz::Panel)