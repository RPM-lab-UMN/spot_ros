#ifndef NAVI_CONTROL_PANEL_H
#define SPOT_CONTROL_PANEL_H

#ifndef Q_MOC_RUN
#  include <ros/ros.h>

#  include <rviz/panel.h>
#endif

#include <map>
#include <QPushButton>
#include <QLabel>
#include <QLCDNumber>
#include <QTextEdit>
#include <QFileDialog>

// TODO add necessary ROS messages

namespace navi_panel
{

class ControlPanel : public rviz::Panel
{
    Q_OBJECT
    public:

    ControlPanel(QWidget *parent=0);
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config& config);

    // Member variables
 private: 
    bool isRecording;

    // Qt Slots
    private Q_SLOTS:
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

    // ROS service clients
    //** None **//

    // ROS topic subscribers
    //** None **//

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