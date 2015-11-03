#include "Gaze.h"

Gaze::Gaze(const std::string & name) :
    as_(nh_, name, false),
    private_node_handle("~"),
    action_name_(name),
    tf_listener(new tf::TransformListener(ros::Duration(40.0))),
    last_fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
    active(false)
{

    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("neck_frame", neck_frame, "neck_frame");
    private_node_handle.param<std::string>("head_origin_frame", head_origin_frame, "head_origin_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");
    private_node_handle.param<std::string>("world_frame", world_frame, "world_frame");

    // Publishers
    fixation_point_goal_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal", 1);
    neck_pan_pub= nh_.advertise<std_msgs::Float64>("/vizzy/neck_pan_position_controller/command", 1);
    neck_tilt_pub=nh_.advertise<std_msgs::Float64>("/vizzy/neck_tilt_position_controller/command", 1);
    eyes_tilt_pub=nh_.advertise<std_msgs::Float64>("/vizzy/eyes_tilt_position_controller/command", 1);
    version_pub=  nh_.advertise<std_msgs::Float64>("/vizzy/version_position_controller/command", 1);
    vergence_pub= nh_.advertise<std_msgs::Float64>("/vizzy/vergence_position_controller/command", 1);

    ROS_INFO("Going to move head and eyes to home position.");

    std_msgs::Float64 neck_pan_angle;
    std_msgs::Float64 neck_tilt_angle;
    std_msgs::Float64 eyes_tilt_angle;
    std_msgs::Float64 vergence_angle;
    std_msgs::Float64 version_angle;
    neck_pan_angle.data=0.0;
    neck_tilt_angle.data=0.0;
    eyes_tilt_angle.data=0.0;
    vergence_angle.data=0.0;
    version_angle.data=0.0;
    neck_pan_pub.publish(neck_pan_angle);
    neck_tilt_pub.publish(neck_tilt_angle);
    eyes_tilt_pub.publish(eyes_tilt_angle);
    vergence_pub.publish(vergence_angle);
    version_pub.publish(version_angle);

    sleep(5.0);

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

    // Subscribers
    neck_pan_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/neck_pan_position_controller/state", 10));
    neck_tilt_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/neck_tilt_position_controller/state", 10));
    eyes_tilt_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/eyes_tilt_position_controller/state", 10));
    version_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/version_position_controller/state", 10));
    vergence_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/vergence_position_controller/state", 10));
    fixation_point_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > (new message_filters::Subscriber<geometry_msgs::PointStamped>(nh_, "fixation_point", 10));
    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
                                                                                                                          *neck_pan_sub,
                                                                                                                          *neck_tilt_sub,
                                                                                                                          *eyes_tilt_sub,
                                                                                                                          *version_sub,
                                                                                                                          *vergence_sub,
                                                                                                                          *fixation_point_sub));

    sync->registerCallback(boost::bind(&Gaze::analysisCB, this, _1, _2, _3, _4, _5, _6));

    as_.registerGoalCallback(boost::bind(&Gaze::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Gaze::preemptCB, this));

    as_.start();
}

bool Gaze::move(const geometry_msgs::PointStamped  &goal_)
{
    ROS_INFO("move");

    std_msgs::Float64 neck_pan_angle;
    std_msgs::Float64 neck_tilt_angle;
    std_msgs::Float64 eyes_tilt_angle;
    std_msgs::Float64 vergence_angle;
    std_msgs::Float64 version_angle;

    eyes_tilt_angle.data=0.0;
    version_angle.data=0.0;

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

    fixation_point(0)=goal_.point.x;
    fixation_point(1)=goal_.point.y;
    fixation_point(2)=goal_.point.z;
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


    neck_pan_pub.publish(neck_pan_angle);
    neck_tilt_pub.publish(neck_tilt_angle);
    eyes_tilt_pub.publish(eyes_tilt_angle);
    vergence_pub.publish(vergence_angle);
    version_pub.publish(version_angle);

    return true;

}

void Gaze::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void Gaze::goalCB()
{
    goal_msg = as_.acceptNewGoal();
    ROS_INFO_STREAM(action_name_.c_str()<<": Received the following goal: "<<*goal_msg);

    start_time = ros::WallTime::now();

    // Convert to neck frame for convenience
    geometry_msgs::PointStamped goal_point;
    geometry_msgs::PointStamped goal_point_world_viz;

    while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(neck_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(neck_frame, current_time, goal_msg->fixation_point, world_frame, goal_point);

            tf_listener->waitForTransform(world_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(world_frame, current_time, goal_msg->fixation_point, world_frame, goal_point_world_viz);
        }
        catch (tf::TransformException &ex)
        {
            continue;
        }
        break;
    }

    fixation_point_goal_pub.publish(goal_point_world_viz);

    if(!move(goal_point))
    {
        result_.state_reached=false;

        ROS_INFO("%s: Aborted", action_name_.c_str());
        as_.setAborted(result_);
    }
    // set the action state to succeeded

    active=true;
    return;
}

void Gaze::analysisCB(const control_msgs::JointControllerState::ConstPtr & neck_pan_msg,
                      const control_msgs::JointControllerState::ConstPtr & neck_tilt_msg,
                      const control_msgs::JointControllerState::ConstPtr & eyes_tilt_msg,
                      const control_msgs::JointControllerState::ConstPtr & version_msg,
                      const control_msgs::JointControllerState::ConstPtr & vergence_msg,
                      const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg)
{
    if(!active)
        return;

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
            tf_listener->waitForTransform(world_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(world_frame, current_time, goal_msg->fixation_point, world_frame, goal_point_);
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

    if(error<goal_msg->fixation_point_error_tolerance)
    {
        result_.state_reached=true;
        result_.fixation_point=fixation_point_;
        result_.fixation_point_error=error;
        feedback_.state_reached=true;

        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(result_);

        ros::WallTime total_time = ros::WallTime::now();
        ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());
        active=false;
    }
    /*else
    {
        ROS_INFO("%s: Active", action_name_.c_str());
    }*/
    as_.publishFeedback(feedback_);
}
