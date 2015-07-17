#include "GazeClient.h"


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_gaze");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<move_robot_msgs::GazeAction> ac("gaze", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    move_robot_msgs::GazeGoal goal;
    goal.fixation_point.point.x = 0.0;
    goal.fixation_point.point.y = 0.0;
    goal.fixation_point.point.z = 0.3;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}
