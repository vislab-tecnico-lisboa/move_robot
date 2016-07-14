// By: Rui P. de Figueiredo : ruifigueiredo@isr.ist.utl.pt

#include "Gaze.h"

Gaze::Gaze(const std::string & name, const ros::NodeHandle & nh) :
    nh_(nh),
    as_(nh_, name, false),
    private_node_handle("~"),
    action_name_(name),
    tf_listener(new tf::TransformListener(ros::Duration(40.0))),
    last_fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
    oculocephalic_group(new moveit::planning_interface::MoveGroup("oculocephalic")),
    active(false),
    it(private_node_handle),
    first_suppresion(true)
{
    while(ros::ok())
    {
        ROS_ERROR("FODAAAAAAAAAAAAAAAAAAAA-SE");
    }
    oculocephalic_joint_names=oculocephalic_group->getActiveJoints();
    oculocephalic_joint_values.resize(oculocephalic_joint_names.size());
    std::fill(oculocephalic_joint_values.begin(), oculocephalic_joint_values.end(), 0);

    private_node_handle.param<std::string>("base_frame", base_frame_id, "base_frame");
    private_node_handle.param("vel_threshold", vel_threshold, 0.1);

    fixation_point_goal_viz_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal_viz", 1);

    ROS_ERROR("OLA");
    // Saccadic
    /*left_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(private_node_handle, "left_camera_in", 1000));
    right_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(private_node_handle, "right_camera_in", 1000));
    joint_state_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > (new message_filters::Subscriber<sensor_msgs::JointState>(private_node_handle, "joint_states", 1000));

    //TF's synchronized with the image
    left_image_filter=boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > (new tf::MessageFilter<sensor_msgs::Image>(*left_image_sub, *tf_listener, base_frame_id, 1000));
    right_image_filter=boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > (new tf::MessageFilter<sensor_msgs::Image>(*right_image_sub, *tf_listener, base_frame_id, 1000));

    gaze_sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100),
                                                                                                                               *left_image_filter,
                                                                                                                               *right_image_filter,
                                                                                                                               *joint_state_sub));

    //gaze_sync->registerCallback(boost::bind(&Gaze::suppresion, this, _1, _2, _3));

    left_image_suppression_pub=nh_.advertise<geometry_msgs::PointStamped>("left_camera_out", 1);
    right_image_suppression_pub=nh_.advertise<geometry_msgs::PointStamped>("right_camera_out", 1);*/
    ROS_ERROR("ADEUS");

}

void Gaze::suppresion(const sensor_msgs::Image::ConstPtr & left_image_msg,
                      const sensor_msgs::Image::ConstPtr & right_image_msg,
                      const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
    ROS_INFO("ENTREI");

    // Get indices
    if (first_suppresion)
    {
        for(int i=0; i<joint_state_msg->name.size();++i)
        {
            if(joint_state_msg->name[i]=="neck_pan_joint")
            {
                joints_to_indices["neck_pan_joint"]=i;
            }
            else if(joint_state_msg->name[i]=="neck_tilt_joint")
            {
                joints_to_indices["neck_tilt_joint"]=i;
            }
            else if(joint_state_msg->name[i]=="r_eye_joint")
            {
                joints_to_indices["r_eye_joint"]=i;
            }
        }

        first_suppresion=false;
    }

    double n_pan=joint_state_msg->velocity[joints_to_indices["neck_pan_joint"]];
    double n_tilt=joint_state_msg->velocity[joints_to_indices["neck_tilt_joint"]];
    double eye=joint_state_msg->velocity[joints_to_indices["r_eye_joint"]];

    double sum_vels=fabs(n_pan+n_tilt+eye); // e_talt = n_pan n_tilt e_pan e_tilt

    if(sum_vels>vel_threshold)
    {
        ROS_INFO("ESTOU A SUPPRIMIR");
        return;
    }

    left_image_suppression_pub.publish(left_image_msg);
    right_image_suppression_pub.publish(right_image_msg);
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


