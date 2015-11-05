#ifndef GAZESIM_H
#define GAZESIM_H

#include "Gaze.h"

class GazeSimulation : public Gaze
{
public:
    GazeSimulation(const std::string & name);
    bool moveHome();
    bool moveCartesian();
    void analysisCB(const control_msgs::JointControllerState::ConstPtr & neck_pan_msg,
                    const control_msgs::JointControllerState::ConstPtr & neck_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & eyes_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & version_msg,
                    const control_msgs::JointControllerState::ConstPtr & vergence_msg,
                    const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);

};

#endif // GAZE_H
