#ifndef NAVI_CONTROL_PANEL_H
#define SPOT_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
#  include <ros/ros.h>

#  include <rviz/panel.h>
#endif

#include <map>
#include <QPushButton>
#include <QLabel>
#include <QTime>
#include <QLCDNumber>
#include <QTextEdit>

#include <spot_msgs/ListGraph.h>
#include <spot_msgs/GraphRecording.h>

namespace navi_panel
{

class ControlPanel : public rviz::Panel
{
    Q_OBJECT
    public:

    ControlPanel(QWidget *parent=0);
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config& config);
    bool callGraphRecordingServices(ros::ServiceClient service, std::string serviceName, spot_msgs::GraphRecording serviceRequest);

    // Member variables
 private: 
    bool isRecording;
    QTimer* recTimer;
    QTime recElapsedTime;

    // Qt Slots
    private Q_SLOTS:
        void tick();
        void recordingToggle();
        void waypointToggle();
        void pointcloudToggle();
        void waypointNav();
        void graphLoad();
        void graphSave();

 private:
    // Component setup functions
    void setupRecordingPanel();
    void setupToggleButtons();
    void setupModalButtons();
    void setupStatusBox();

    // Custom functions
    void logStatus(QString msg);

    // ROS services
    ros::NodeHandle nh_;
    ros::ServiceClient startRecordingService_;
    ros::ServiceClient stopRecordingService_;
    ros::ServiceClient getRecordingStatusService_;
    ros::ServiceClient downloadGraphService_;
    ros::ServiceClient uploadGraphService_;
    ros::ServiceClient listGraphService_;

    // ROS topic subscribers
    ros::Subscriber graphWaypointsSub_;
    ros::Subscriber graphEdgesSub_;

    // QT UI Widgets
    QPushButton* recordingToggleButton;
    QPushButton* waypointToggleButton;
    QPushButton* pointcloudToggleButton;
    QPushButton* waypointNavButton;
    QPushButton* graphLoadButton;
    QPushButton* graphSaveButton;

    QLCDNumber* recordingTime;

    QTextEdit* statusBox;


}; // end class ControlPanel

} // end namespace navi_panel

#endif