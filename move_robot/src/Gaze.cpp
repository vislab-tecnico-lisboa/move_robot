// By: Rui P. de Figueiredo : ruifigueiredo@isr.ist.utl.pt

#include "Gaze.h"

Gaze::Gaze(const std::string & name) :
    as_(nh_, name, false),
    private_node_handle("~"),
    action_name_(name),
    tf_listener(new tf::TransformListener(ros::Duration(40.0))),
    last_fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
    oculocephalic_group(new moveit::planning_interface::MoveGroup("oculocephalic")),
    active(false)
{
    oculocephalic_joint_names=oculocephalic_group->getActiveJoints();
    oculocephalic_joint_values.resize(oculocephalic_joint_names.size());
    std::fill(oculocephalic_joint_values.begin(), oculocephalic_joint_values.end(), 0);

    fixation_point_goal_viz_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal_viz", 1);
}

void Gaze::publishFixationPointGoal()
{
    // Convert to neck frame for convenience
    geometry_msgs::PointStamped goal_point_world_viz;
    while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(world_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(world_frame, current_time, goal_msg->fixation_point, world_frame, goal_point_world_viz);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    fixation_point_goal_viz_pub.publish(goal_point_world_viz);
}


void Gaze::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    active=false;
    as_.setPreempted();
}

void Gaze::goalCB()
{
    goal_msg = as_.acceptNewGoal();
    ROS_INFO_STREAM(action_name_.c_str()<<": Received the following goal: "<<*goal_msg);

    start_time = ros::WallTime::now();
    active=true;

    if(goal_msg->type==move_robot_msgs::GazeGoal::HOME)
    {
        moveHome();
    }
    else
    {
        publishFixationPointGoal();
        if(!moveCartesian())
        {
            result_.state_reached=false;

            ROS_INFO("%s: Aborted", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
    // set the action state to succeeded

    return;
}


