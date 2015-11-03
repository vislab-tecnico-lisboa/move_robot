#include "Gaze.h"

Gaze::Gaze(const std::string & name) :
    as_(nh_, name, false),
    private_node_handle("~"),
    action_name_(name),
    tf_listener(new tf::TransformListener(ros::Duration(40.0))),
    robot_model_loader("robot_description"),
    robot_model(robot_model_loader.getModel()),
    oculocephalic_joint_model_group(robot_model->getJointModelGroup("oculocephalic")),
    oculocephalic_group(new moveit::planning_interface::MoveGroup("oculocephalic")),
    //eyes_joint_model_group(robot_model->getJointModelGroup("eyes")),
    //eyes_group(new moveit::planning_interface::MoveGroup("eyes")),
    last_fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()))
{
    nh_.setParam("/move_group/trajectory_execution/execution_duration_monitoring", false);
    nh_.setParam("/move_group/trajectory_execution/allowed_execution_duration_scaling",1000.0);
    oculocephalic_joint_names=oculocephalic_group->getActiveJoints();
    oculocephalic_joint_values.resize(oculocephalic_joint_names.size());
    std::cout << oculocephalic_joint_names[0] << " "
                                              << oculocephalic_joint_names[1] << " "
                                              << oculocephalic_joint_names[2] << " "
                                              << oculocephalic_joint_names[3] <<" "
                                              << oculocephalic_joint_names[4] << std::endl;

    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("neck_frame", neck_frame, "neck_frame");
    private_node_handle.param<std::string>("head_origin_frame", head_origin_frame, "head_origin_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");
    private_node_handle.param<std::string>("world_frame", world_frame, "world_frame");

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    fixation_point_goal_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal", 1);

    ROS_INFO("Going to move head and eyes to home position.");
    oculocephalic_group->setNamedTarget("oculocephalic_home");
    while(!oculocephalic_group->move()&&nh_.ok());
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
    fixation_point_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > (new message_filters::Subscriber<geometry_msgs::PointStamped>(nh_, "fixation_point", 10));
    move_group_action_feedback_sub=boost::shared_ptr<message_filters::Subscriber<moveit_msgs::MoveGroupActionFeedback> > (new message_filters::Subscriber<moveit_msgs::MoveGroupActionFeedback>(nh_, "/move_group/feedback", 10));
    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *fixation_point_sub, *move_group_action_feedback_sub));
    sync->registerCallback(boost::bind(&Gaze::analysisCB, this, _1, _2));

    as_.registerGoalCallback(boost::bind(&Gaze::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Gaze::preemptCB, this));

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

    oculocephalic_joint_values[0] = neck_pan_angle.data;
    oculocephalic_joint_values[1] = neck_tilt_angle.data;
    oculocephalic_joint_values[2] = 0;

    oculocephalic_joint_values[3] = vergence_angle.data;
    oculocephalic_joint_values[4] = 0;

    if(!oculocephalic_group->setJointValueTarget(oculocephalic_joint_values))//||!eyes_group->setJointValueTarget(eyes_joint_values))
    {
        ROS_WARN("Fixation point out of head working space!");
        publishFixationPoint(fixation_point,goal.header.frame_id,false);

        return false;
    }
    else
    {
        publishFixationPoint(fixation_point,goal.header.frame_id,true);

        last_fixation_point=fixation_point;
        result_.fixation_point=goal;
        result_.fixation_point.header.stamp=ros::Time::now();
        ROS_INFO("Going to move head and eyes...");
        if(!oculocephalic_group->asyncMove())
            return false;
        ROS_INFO("Done.");


        //ROS_INFO("Going to move head...");
        //if(!head_group->move())
        //    return false;
        //ROS_INFO("Done.");
        return true;
    }
}

void Gaze::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void Gaze::goalCB()
{
    goal = as_.acceptNewGoal();
    ROS_INFO_STREAM(action_name_.c_str()<<": Received the following goal: "<<*goal);

    start_time = ros::WallTime::now();

    // Convert to neck frame for convenience
    geometry_msgs::PointStamped goal_point;
    while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(neck_frame, current_time, goal->fixation_point.header.frame_id, goal->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(neck_frame, current_time, goal->fixation_point, world_frame, goal_point);
        }
        catch (tf::TransformException &ex)
        {
            continue;
        }
        break;
    }

    if(!move(goal_point))
    {
        result_.state_reached=false;

        ROS_INFO("%s: Aborted", action_name_.c_str());
        as_.setAborted(result_);
    }
    // set the action state to succeeded




    return;
}

void Gaze::analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg, const moveit_msgs::MoveGroupActionFeedback::ConstPtr &move_group_action_feedback_msg)
{

    // Convert points to world frame
    geometry_msgs::PointStamped fixation_point_;
    geometry_msgs::PointStamped goal_point_;

    while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(world_frame, current_time, fixation_point_msg->header.frame_id, fixation_point_msg->header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(world_frame, current_time, *fixation_point_msg, world_frame, fixation_point_);
            tf_listener->waitForTransform(world_frame, current_time, goal->fixation_point.header.frame_id, goal->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(world_frame, current_time, goal->fixation_point, world_frame, goal_point_);
        }
        catch (tf::TransformException &ex)
        {
            continue;
        }
        break;
    }

    double error_x=fixation_point_.point.x-goal_point_.point.x;
    double error_y=fixation_point_.point.y-goal_point_.point.y;
    double error_z=fixation_point_.point.z-goal_point_.point.z;
    double error=sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
    feedback_.state_reached=false;
    feedback_.fixation_point=fixation_point_;
    feedback_.fixation_point_error=error;

    if(move_group_action_feedback_msg->status.status==actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        result_.state_reached=true;
        result_.fixation_point=fixation_point_;
        result_.fixation_point_error=error;
        feedback_.state_reached=true;

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);

        ros::WallTime total_time = ros::WallTime::now();
        ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());
    }
    else if(move_group_action_feedback_msg->status.status==actionlib_msgs::GoalStatus::ABORTED)
    {
        result_.state_reached=false;
        result_.fixation_point=fixation_point_;
        result_.fixation_point_error=error;

        ROS_WARN("%s: Aborted", action_name_.c_str());
        as_.setAborted(result_);

        ros::WallTime total_time = ros::WallTime::now();
        ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());
    }
    else if(move_group_action_feedback_msg->status.status==actionlib_msgs::GoalStatus::PREEMPTED)
    {
        result_.state_reached=false;
        result_.fixation_point=*fixation_point_msg;
        result_.fixation_point_error=error;

        ROS_WARN("%s: Preempted", action_name_.c_str());
        as_.setPreempted(result_);

        ros::WallTime total_time = ros::WallTime::now();
        ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());
    }
    else if(move_group_action_feedback_msg->status.status==actionlib_msgs::GoalStatus::PENDING)
    {
        ROS_WARN("%s: Pending", action_name_.c_str());
    }
    /*else if(move_group_action_feedback_msg->status.status==actionlib_msgs::GoalStatus::ACTIVE)
    {
        ROS_INFO("%s: Active", action_name_.c_str());
    }*/
    as_.publishFeedback(feedback_);
}
