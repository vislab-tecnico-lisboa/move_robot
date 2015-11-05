#include "GazeReal.h"

GazeReal::GazeReal(const std::string & name) : Gaze(name)
{}


// TODO
bool GazeReal::moveHome()
{

}

bool GazeReal::moveCartesian()
{

}

void GazeReal::analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg)
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    GazeReal gaze(ros::this_node::getName());
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

