#include "navi_panel.hpp"

#include <QFile>
#include <QUiLoader>
#include <QVBoxLayout>
#include <ros/package.h>
#include <string.h>


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

        // Setup ROS service clients
        //** None **//

        // Get Qt Widget handles
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        waypointToggleButton = this->findChild<QPushButton*>("waypointToggleButton");
        pointcloudToggleButton = this->findChild<QPushButton*>("pointcloudToggleButton");
        waypointNavButton = this->findChild<QPushButton*>("waypointNavButton");
        graphLoadButton = this->findChild<QPushButton*>("graphLoadButton");
        graphSaveButton = this->findChild<QPushButton*>("graphSaveButton");

        recordingStatus = this->findChild<QLabel*>("recordingStatus");

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");

        // Call Widget setup functions
        setupRecordingPanel();
        setupToggleButtons();
        setupModalButtons();
        setupStatusBox();

        // Setup ROS topic subscribers now that everything is set up
        //** None **//

        // TODO Connect Qt Signals and Slots
    }

    // Setup functions
    void ControlPanel::setupRecordingPanel() {
        // TODO
    }

    void ControlPanel::setupToggleButtons() {
        // TODO
    }

    void ControlPanel::setupModalButtons() {
        // TODO
    }

    void ControlPanel::setupStatusBox() {
        // TODO
    }

    // Custom functions
    //** None **//

    // ROS message callbacks
    //** None **//

    // Qt slot functions
    void ControlPanel::recordingToggle() {
        // TODO
    }

    void ControlPanel::waypointToggle() {
        // TODO
    }

    void ControlPanel::pointcloudToggle() {
        // TODO
    }

    void ControlPanel::waypointNav() {
        // TODO
    }

    void ControlPanel::graphLoad() {
        // TODO
    }

    void ControlPanel::graphSave() {
        // TODO
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