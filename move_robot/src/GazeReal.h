#ifndef GAZEREAL_H
#define GAZEREAL_H

#include "Gaze.h"

class GazeReal : public Gaze
{
protected:
    ros::Publisher gazePublisher;
    ros::Subscriber fix_point_sub;
    std::string base_frame;

public:
    GazeReal(const std::string & name);
    bool moveHome();
    bool moveCartesian();
    void analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);
};

#endif // GAZEREAL_H
