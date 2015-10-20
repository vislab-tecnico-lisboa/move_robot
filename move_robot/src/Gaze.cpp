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

    //state_monitor=boost::shared_ptr<planning_scene_monitor::CurrentStateMonitor>(new planning_scene_monitor::CurrentStateMonitor(robot_model,tf_listener));
    //state_monitor->startStateMonitor("/vizzy/joint_states");

    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("neck_frame", neck_frame, "neck_frame");
    private_node_handle.param<std::string>("head_origin_frame", head_origin_frame, "head_origin_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    fixation_point_goal_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal", 1);

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
            tf_listener->waitForTransform(head_origin_frame, neck_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(head_origin_frame, neck_frame, ros::Time(0), transform);
            tf::Vector3 origin;
            origin=transform.getOrigin();
            y_offset=origin.getY();

            tf_listener->waitForTransform(eyes_center_frame, head_origin_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(eyes_center_frame, head_origin_frame, ros::Time(0), transform);
            origin=transform.getOrigin();
            z_offset=origin.getZ();
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }



    as_.start();
}

void Gaze::publishFixationPoint(const Eigen::Vector3d &goal, const std::string & frame_id, const bool valid)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id=frame_id;
    geometry_msgs::PointStamped fixation_point_msg;
    fixation_point_msg.header.frame_id=frame_id;

    fixation_point_msg.point.x=goal(0);
    fixation_point_msg.point.y=goal(1);
    fixation_point_msg.point.z=goal(2);

    fixation_point_goal_pub.publish(fixation_point_msg);
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
        double x=fixation_point.x();
        double y=fixation_point.y();
        double z=fixation_point.z();

        double neck_pan_angle_=atan2(x,z);
        double tilt_angle;
        double aux=(-y*y_offset + sqrt((x*x + z*z)*(x*x + y*y - y_offset*y_offset + z*z)));
        if(y<-y_offset)
            tilt_angle=acos(aux/(fixation_point.squaredNorm()));
        else
            tilt_angle=-acos(aux/(fixation_point.squaredNorm()));

        neck_pan_angle.data=neck_pan_angle_; // Good
        neck_tilt_angle.data=tilt_angle;
        vergence_angle.data=M_PI-2.0*atan2(fixation_point.norm()*cos(asin(y_offset/fixation_point.norm()))+z_offset,half_base_line);
    }

    head_joint_values[0] = neck_pan_angle.data;
    head_joint_values[1] = neck_tilt_angle.data;
    head_joint_values[2] = 0;

    eyes_joint_values[0] = vergence_angle.data;
    eyes_joint_values[1] = 0;

    if(!head_group->setJointValueTarget(head_joint_values)||!eyes_group->setJointValueTarget(eyes_joint_values))
    {
        ROS_WARN("Fixation point out of head working space!");
        publishFixationPoint(fixation_point,goal.header.frame_id,false);

        /*
         *        Eigen::Vector3d fixation_point_perturb;
         * do
        {
            publishFixationPoint(fixation_point_perturb,goal.header.frame_id,false);

            cv::Mat aux(1, 1, CV_64F);

            // Generate random patch on the sphere surface
            cv::randn(aux, 0, 0.1);
            fixation_point_perturb(0,0)=fixation_point.x()+aux.at<double>(0,0);

            cv::randn(aux, 0, 0.1);
            fixation_point_perturb(1,0)=fixation_point.y()+aux.at<double>(0,0);

            cv::randn(aux, 0, 0.1);
            fixation_point_perturb(2,0)=fixation_point.z()+aux.at<double>(0,0);
            //cv::randn(aux, 0, 0.1);
            //fixation_point_perturb= fixation_point_normalized*aux.at<double>(0,0)+fixation_point;
            Eigen::Vector3d fixation_point_perturb_normalized=fixation_point_perturb.normalized();

            if(fixation_point_perturb_normalized.x()!=fixation_point_perturb_normalized.x())
            {
                neck_pan_angle.data=0.0;
                neck_tilt_angle.data=0.0;
                vergence_angle.data=0.0;
            }
            else
            {
                neck_pan_angle.data=atan2(fixation_point_perturb.x(),fixation_point_perturb.z());
                double tilt_angle=-atan2(fixation_point_perturb.y()+y_offset,sqrt((fixation_point_perturb.x()*fixation_point_perturb.x())+(fixation_point_perturb.z()*fixation_point_perturb.z())));
                neck_tilt_angle.data=tilt_angle;
                vergence_angle.data=M_PI-2*atan2(fixation_point_perturb.norm()+sin(tilt_angle)*y_offset+z_offset,half_base_line);
            }

            head_joint_values[0] = neck_pan_angle.data;
            head_joint_values[1] = neck_tilt_angle.data;
            head_joint_values[2] = 0.0;

            eyes_joint_values[0] = vergence_angle.data;
            eyes_joint_values[1] = 0.0;
        }
        while(!head_group->setJointValueTarget(head_joint_values)||!eyes_group->setJointValueTarget(eyes_joint_values)&&nh_.ok());
        ROS_WARN("Found good fixation point!");
        publishFixationPoint(fixation_point_perturb,goal.header.frame_id,true);

        result_.fixation_point.header.frame_id=goal.header.frame_id;
        result_.fixation_point.header.stamp=ros::Time::now();
        result_.fixation_point.point.x=fixation_point_perturb.x();
        result_.fixation_point.point.y=fixation_point_perturb.y();
        result_.fixation_point.point.z=fixation_point_perturb.z();*/

        return false;

    }
    else
    {
        result_.fixation_point=goal;
        result_.fixation_point.header.stamp=ros::Time::now();
        publishFixationPoint(fixation_point,goal.header.frame_id,true);
    }

    ROS_INFO("Going to move eyes...");
    eyes_group->move();
    ROS_INFO("Done.");


    ROS_INFO("Going to move head...");
    head_group->move();
    ROS_INFO("Done.");

    return true;
    //state_monitor->getCurrentState()->copyJointGroupPositions(head_joint_model_group, head_joint_values);
}

void Gaze::executeCB(const move_robot_msgs::GazeGoalConstPtr &goal)
{
    ROS_INFO_STREAM(action_name_.c_str()<<": Received the following goal: "<<*goal);

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
                tf_listener->waitForTransform(neck_frame, goal->fixation_point.header.frame_id, ros::Time(0), ros::Duration(10.0) );
                tf_listener->transformPoint(neck_frame, goal->fixation_point, goal_point);
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
            result_.state_reached=true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
        else
        {
            result_.state_reached=false;

            ROS_INFO("%s: Aborted", action_name_.c_str());
            as_.setAborted(result_);
        }
        // set the action state to succeeded
    }

    ros::WallTime total_time = ros::WallTime::now();

    ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());

    return;
}

