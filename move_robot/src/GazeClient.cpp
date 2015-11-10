#include "GazeClient.h"


int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_gaze");

    ros::NodeHandle nh;
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<move_robot_msgs::GazeAction> ac("gaze", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    double rate=10.0;
    ros::Rate r(rate);

double rate_aux=20.0;

    // send a goal to the action
    int i=0;
    while(nh.ok())
    {
        ++i;

        double angular_freq=2*M_PI/rate_aux;
        double time_instant=(double)i;
        double aux=angular_freq*time_instant;
        std::cout << aux << " "<<cos(aux) <<std::endl;
        move_robot_msgs::GazeGoal goal;
        goal.type=move_robot_msgs::GazeGoal::CARTESIAN;
        goal.fixation_point.point.x = 0.2*cos(aux);
        goal.fixation_point.point.y =  0.4*cos(aux);
        goal.fixation_point.point.z = 0.75+0.25*cos(aux);
        goal.fixation_point_error_tolerance = 0.005;

        //goal.fixation_point.point.z = 0.5;

        goal.fixation_point.header.frame_id="ego_frame";
        goal.fixation_point.header.stamp=ros::Time::now();

        ac.sendGoal(goal);
        ROS_INFO("Action server started, sending goal.");

        //wait for the action to return
        //bool finished_before_timeout = ac.waitForResult(ros::Duration(1.0));

        /*if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");*/

        r.sleep();

    }
    //exit
    return 0;
}
