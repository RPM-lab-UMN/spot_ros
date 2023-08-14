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
#include <std_msgs/Trigger.h>
#include <std_msgs/TriggerResponse.h>
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
        listGraphService_ = nh_.serviceClient<spot_msgs::ListGraph>("/spot/list_graph");
        clearGraphService_ = nh_.serviceClient<spot_msgs::GraphRecording>("/spot/clear_graph");
        clearWaypointService_ = nh_.serviceClient<std_msgs::Trigger>("/spot/graph_waypoints/clear");

        // setup ROS action client for navigation commands
        

        // Get Qt Widget handles
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        listGraphButton = this->findChild<QPushButton*>("listGraphButton");
        pointcloudToggleButton = this->findChild<QPushButton*>("pointcloudToggleButton");
        clearGraphButton = this->findChild<QPushButton*>("clearGraphButton");
        graphLoadButton = this->findChild<QPushButton*>("graphLoadButton");
        graphSaveButton = this->findChild<QPushButton*>("graphSaveButton");

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");

        statusBox = this->findChild<QTextEdit*>("statusBox");

        // Call Widget setup functions
        setupRecordingPanel();
        setupToggleButtons();
        setupModalButtons();
        setupStatusBox();



        // Connect Qt Signals and Slots
        connect(recordingToggleButton, SIGNAL(clicked()), this, SLOT(recordingToggle()));
        connect(recTimer, SIGNAL(timeout()), this, SLOT(tick()));
        connect(listGraphButton, SIGNAL(clicked()), this, SLOT(listGraph()));
        connect(pointcloudToggleButton, SIGNAL(clicked()), this, SLOT(pointcloudToggle()));
        connect(clearGraphButton, SIGNAL(clicked()), this, SLOT(clearGraph()));
        connect(graphLoadButton, SIGNAL(clicked()), this, SLOT(graphLoad()));
        connect(graphSaveButton, SIGNAL(clicked()), this, SLOT(graphSave()));
    }

    // Setup functions
    void ControlPanel::setupRecordingPanel() {
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        spot_msgs::GraphRecording initalRecordingState;
        initalRecordingState.request.path = std::string("");
        // check whether robot is already recording
        callGraphRecordingServices(getRecordingStatusService_, "get recording status", initalRecordingState);
        if (initalRecordingState.response.success) {
            isRecording = true;
            recordingToggleButton->setText(QString::fromUtf8("Stop Recording"));
        }
        else {
            isRecording = false;
            recordingToggleButton->setText(QString::fromUtf8("Start Recording"));
        }
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
        listGraphButton = this->findChild<QPushButton*>("listGraphButton");

        listGraphButton->setText(QString::fromUtf8("List Graph Waypoints"));
        listGraphButton->update();

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

        clearGraphButton = this->findChild<QPushButton*>("clearGraphButton");
        QPalette pal = clearGraphButton->palette();
        pal.setColor(QPalette::Button, QColor(85, 90, 255));
        clearGraphButton->setAutoFillBackground(true);
        clearGraphButton->setPalette(pal);
        clearGraphButton->update();
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
    bool ControlPanel::callListGraphService(spot_msgs::ListGraph serviceRequest) {
        std::string labelText = "Calling list graph service";
        statusBox->setText(QString(labelText.c_str()));
        return listGraphService_.call(serviceRequest);
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

    void ControlPanel::listGraph() {
        logStatus(QString::fromUtf8("listing graph ..."));
        spot_msgs::ListGraph req;
        callListGraphService(req);
        std::vector<std::string> waypoints_list = req.response.waypoint_ids;
        QString joined_waypoints = "";
        for (int i = 0; i < waypoints_list.size(); i++) {
            joined_waypoints += QString::fromUtf8((waypoints_list.at(i) + std::string("\n")).c_str());
        }
        statusBox->setText(joined_waypoints);

        return;
    }

    void ControlPanel::pointcloudToggle() {
        logStatus(QString::fromUtf8("Toggling pointcloud visibility..."));

        // TODO toggle pointcloud display

        return;
    }

    void ControlPanel::clearGraph() {
        logStatus(QString::fromUtf8("clearing graph ..."));
        spot_msgs::GraphRecording req;
        req.request.path = std::string("");
        callGraphRecordingServices(clearGraphService_, "clear graph", req);
        clearWaypointService_.call()

        return;
    }

    void ControlPanel::graphLoad() {
        logStatus(QString::fromUtf8("Opening file browser..."));

        QString dir = QFileDialog::getExistingDirectory(this, "Load Graph from Directory", QDir::currentPath(), QFileDialog::ShowDirsOnly | QFileDialog::ReadOnly);
        logStatus(dir);

        spot_msgs::GraphRecording req;
        req.request.path = dir.toStdString();
        callGraphRecordingServices(uploadGraphService_, "upload recording", req);

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
        req.request.path = dirPath.toStdString() + std::string("/downloaded_graph");

        callGraphRecordingServices(downloadGraphService_, "save graph", req);    

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