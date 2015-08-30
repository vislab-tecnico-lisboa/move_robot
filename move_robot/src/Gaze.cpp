#include "Gaze.h"

Gaze::Gaze(const std::string & name) :
    as_(nh_, name, boost::bind(&Gaze::executeCB, this, _1), false),
    private_node_handle("~"),
    action_name_(name),
    tf_listener(new tf::TransformListener(ros::Duration(2.0))),
    robot_model_loader("robot_description"),
    robot_model(robot_model_loader.getModel()),
    head_joint_model_group(robot_model->getJointModelGroup("head")),
    head_group(new moveit::planning_interface::MoveGroup("head")),
    eyes_joint_model_group(robot_model->getJointModelGroup("eyes")),
    eyes_group(new moveit::planning_interface::MoveGroup("eyes"))
{
    nh_.setParam("/move_group/trajectory_execution/execution_duration_monitoring", false);
    nh_.setParam("/move_group/trajectory_execution/allowed_execution_duration_scaling",1000.0);
    head_joint_names=head_group->getActiveJoints();
    head_joint_values.resize(head_joint_names.size());
    std::cout << head_joint_names[0] << " " << head_joint_names[1] << " " << head_joint_names[2] << std::endl;

    eyes_joint_names=eyes_group->getActiveJoints();
    eyes_joint_values.resize(eyes_joint_names.size());
    std::cout << eyes_joint_names[0] << " " << eyes_joint_names[1] << std::endl;

    state_monitor=boost::shared_ptr<planning_scene_monitor::CurrentStateMonitor>(new planning_scene_monitor::CurrentStateMonitor(robot_model,tf_listener));
    state_monitor->startStateMonitor("/vizzy/joint_states");

    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());
    //kinematic_state->setToDefaultValues();

    fixation_point_marker_pub = nh_.advertise<visualization_msgs::Marker>("fixation_point", 1);
    gaze_arrow_marker_pub = nh_.advertise<visualization_msgs::Marker>("gaze_arrow", 1);

    ROS_INFO("Going to move head to home position.");
    eyes_group->setNamedTarget("eyes_home");
    eyes_group->move();

    ROS_INFO("Going to move eyes to home position.");
    head_group->setNamedTarget("head_home");
    head_group->move();
    ROS_INFO("Done.");


    tf::StampedTransform transform;


    while(nh_.ok())
    {
        try
        {
            tf_listener->waitForTransform(eyes_center_frame, ego_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(eyes_center_frame, ego_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    tf::Vector3 origin;

    origin=transform.getOrigin();
    y_offset=origin.getY();
    z_offset=origin.getZ();

    as_.start();
}

void Gaze::publishFixationPoint(const move_robot_msgs::GazeGoalConstPtr &goal, bool valid)
{
    visualization_msgs::Marker marker;
    marker.header=goal->fixation_point.header;
    marker.header.stamp = ros::Time();
    marker.ns = "fixation_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal->fixation_point.point.x;
    marker.pose.position.y = goal->fixation_point.point.y;
    marker.pose.position.z = goal->fixation_point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    if(valid)
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }

    fixation_point_marker_pub.publish(marker);
    marker.type = visualization_msgs::Marker::ARROW;
    gaze_arrow_marker_pub.publish(marker);
}

bool Gaze::move(const geometry_msgs::PointStamped  &goal)
{
    std_msgs::Float64 neck_pan_angle;
    std_msgs::Float64 neck_tilt_angle;
    std_msgs::Float64 vergence_angle;

    tf::StampedTransform transform;

    while(nh_.ok())
    {
        try
        {
            tf_listener->waitForTransform(right_eye_frame, left_eye_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(right_eye_frame, left_eye_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }
    tf::Vector3 origin=transform.getOrigin();
    half_base_line=(double)origin.length()/2.0; // meters

    Eigen::Vector3d fixation_point;

    fixation_point(0)=goal.point.x;
    fixation_point(1)=goal.point.y;
    fixation_point(2)=goal.point.z;
    Eigen::Vector3d fixation_point_normalized=fixation_point.normalized();

    if(fixation_point_normalized.x()!=fixation_point_normalized.x())
    {
        neck_pan_angle.data=0.0;
        neck_tilt_angle.data=0.0;
        vergence_angle.data=0.0;
    }
    else
    {
        neck_pan_angle.data=atan2(fixation_point.x(),fixation_point.z());;
        neck_tilt_angle.data=-atan2(fixation_point.y()+y_offset,sqrt((fixation_point.x()*fixation_point.x())+(fixation_point.z()*fixation_point.z())));
        vergence_angle.data=M_PI/2.0-atan2(fixation_point.norm()+z_offset,half_base_line);
    }

    head_joint_values[0] = neck_pan_angle.data;
    head_joint_values[1] = neck_tilt_angle.data;
    head_joint_values[2] = 0;
    if(!head_group->setJointValueTarget(head_joint_values))
    {
        ROS_WARN("Fixation point out of head working space!");
        return false;
    }
    ROS_INFO("Going to move eyes...");
    head_group->move();
    ROS_INFO("Done.");

    eyes_joint_values[0] = vergence_angle.data;
    eyes_joint_values[1] = 0;
    if(!eyes_group->setJointValueTarget(eyes_joint_values))
    {
        ROS_WARN("Fixation point out of eyes working space!");
        return false;
    }
    ROS_INFO("Going to move eyes...");
    eyes_group->move();
    ROS_INFO("Done.");

    return true;
    //state_monitor->getCurrentState()->copyJointGroupPositions(head_joint_model_group, head_joint_values);
}

void Gaze::executeCB(const move_robot_msgs::GazeGoalConstPtr &goal)
{
    ros::WallTime start_time = ros::WallTime::now();

    // helper variables
    bool success = true;


    if (as_.isPreemptRequested() || !ros::ok())
    {
        result_.state_reached=false;
        as_.setPreempted();
        success=false;
    }

    if(success)
    {
        geometry_msgs::PointStamped goal_point;
        while(nh_.ok())
        {
            try
            {
                tf_listener->waitForTransform(ego_frame, goal->fixation_point.header.frame_id, ros::Time(0), ros::Duration(10.0) );
                tf_listener->transformPoint(ego_frame, goal->fixation_point, goal_point);
            }
            catch (tf::TransformException &ex)
            {
                //ROS_WARN("%s",ex.what());
                continue;
            }
            break;
        }

        if(move(goal_point))
        {
            publishFixationPoint(goal,true);
            result_.state_reached=true;
        }
        else
        {
            publishFixationPoint(goal,false);
            result_.state_reached=false;
        }
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }

    ros::WallTime total_time = ros::WallTime::now();

    ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());

    return;
}

