#include "GazeSimulation.h"

GazeSimulation::GazeSimulation(const std::string & name) : Gaze(name)
{}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    GazeSimulation gaze(ros::this_node::getName());
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}
