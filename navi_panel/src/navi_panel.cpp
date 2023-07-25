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

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");

        statusBox = this->findChild<QTextEdit*>("statusBox");

        // Call Widget setup functions
        setupRecordingPanel();
        setupToggleButtons();
        setupModalButtons();
        setupStatusBox();

        // Setup ROS topic subscribers now that everything is set up
        //** None **//

        // Connect Qt Signals and Slots
        connect(recordingToggleButton, SIGNAL(clicked()), this, SLOT(recordingToggle()));
        connect(waypointToggleButton, SIGNAL(clicked()), this, SLOT(waypointToggle()));
        connect(pointcloudToggleButton, SIGNAL(clicked()), this, SLOT(pointcloudToggle()));
        connect(waypointNavButton, SIGNAL(clicked()), this, SLOT(waypointNav()));
        connect(graphLoadButton, SIGNAL(clicked()), this, SLOT(graphLoad()));
        connect(graphSaveButton, SIGNAL(clicked()), this, SLOT(graphSave()));
    }

    // Setup functions
    void ControlPanel::setupRecordingPanel() {
        recordingToggleButton = this->findChild<QPushButton*>("recordingToggleButton");
        recordingToggleButton->setText(QString::fromUtf8("\u23FA Start Recording"));
        QPalette pal = recordingToggleButton->palette();
        pal.setColor(QPalette::Button, QColor(255, 85, 90));
        recordingToggleButton->setAutoFillBackground(true);
        recordingToggleButton->setPalette(pal);
        recordingToggleButton->update();

        recordingTime = this->findChild<QLCDNumber*>("recordingTime");
        recordingTime->setDigitCount(4);
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
    }

    void ControlPanel::setupStatusBox() {
        statusBox = this->findChild<QTextEdit*>("statusBox");
        statusBox->setReadOnly(true);
        statusBox->setPlainText(QString::fromUtf8("Log messages will appear here."));
        statusBox->update();
    }

    // Custom functions
    //** None **//

    // ROS message callbacks
    //** None **//

    // Qt slot functions
    void ControlPanel::recordingToggle() {
        // TODO
        return;
    }

    void ControlPanel::waypointToggle() {
        // TODO
        return;
    }

    void ControlPanel::pointcloudToggle() {
        // TODO
        return;
    }

    void ControlPanel::waypointNav() {
        // TODO
        return;
    }

    void ControlPanel::graphLoad() {
        // TODO
        return;
    }

    void ControlPanel::graphSave() {
        // TODO
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