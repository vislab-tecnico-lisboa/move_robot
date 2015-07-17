#include "GazeReal.h"
Gaze::Gaze(std::string name) :
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


    std::string left_camera_frame;
    std::string right_camera_frame;
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");
    tf::StampedTransform transform;

    try
    {
        tf_listener->waitForTransform(right_camera_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener->lookupTransform(right_camera_frame, left_camera_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        exit(-1);
    }

    tf::Vector3 origin=transform.getOrigin();

    half_base_line=(double)origin.length()/2.0; // meters

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());
    //kinematic_state->setToDefaultValues();

    fixation_point_publisher = nh_.advertise<geometry_msgs::Point>("/vizzy/x_in", 10);

    as_.start();
}

void Gaze::move(const Eigen::Vector3d & fixation_point)
{
    std_msgs::Float64 neck_pan_angle;
    std_msgs::Float64 neck_tilt_angle;
    std_msgs::Float64 vergence_angle;

    Eigen::Vector3d fixation_point_normalized=fixation_point.normalized();
    if(fixation_point_normalized.x()!=fixation_point_normalized.x())
    {
        neck_pan_angle.data=0.0;
        neck_tilt_angle.data=0.0;
        vergence_angle.data=0.0;
    }
    else
    {
        neck_pan_angle.data=asin(fixation_point_normalized.x());
        neck_tilt_angle.data=asin(fixation_point_normalized.y());
        vergence_angle.data=M_PI/2.0-atan2(fixation_point.z(),half_base_line);
    }

    head_joint_values[0] = neck_pan_angle.data;
    head_joint_values[1] = 0;
    head_joint_values[2] = neck_tilt_angle.data;
    head_group->setJointValueTarget(head_joint_values);
    std::cout << "going to move head..." << std::endl;
    head_group->move();
    std::cout << "done" << std::endl;

    eyes_joint_values[0] = vergence_angle.data;
    eyes_joint_values[1] = 0;
    eyes_group->setJointValueTarget(eyes_joint_values);
    std::cout << "going to move eyes..." << std::endl;
    eyes_group->move();
    std::cout << "done" << std::endl;

    state_monitor->getCurrentState()->copyJointGroupPositions(head_joint_model_group, head_joint_values);
}

void Gaze::executeCB(const move_robot_msgs::GazeGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    Eigen::Vector3d fixation_point_;
    fixation_point_(0)=goal->fixation_point.point.x;
    fixation_point_(1)=goal->fixation_point.point.y;
    fixation_point_(2)=goal->fixation_point.point.z;

    geometry_msgs::Point fixation_point_msg;
    fixation_point_msg.x=goal->fixation_point.point.x;
    fixation_point_msg.y=goal->fixation_point.point.y;
    fixation_point_msg.z=goal->fixation_point.point.z;

    fixation_point_publisher.publish(fixation_point_msg);

    // READ TOPIC...

    success=true;
    if(success)
    {
        result_.state_reached=true;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    else
    {
        result_.state_reached=false;
        ROS_INFO("%s: Didn't succeed", action_name_.c_str());

        as_.setAborted(result_);
    }
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    Gaze gaze(ros::this_node::getName());
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}
