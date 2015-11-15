#include "GazeReal.h"

GazeReal::GazeReal(const std::string & name) : Gaze(name)
{

    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("neck_frame", neck_frame, "neck_frame");
    private_node_handle.param<std::string>("head_origin_frame", head_origin_frame, "head_origin_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");
    private_node_handle.param<std::string>("world_frame", world_frame, "world_frame");
    private_node_handle.param<std::string>("fixation_point_frame", fixation_point_frame, "fixation_point_frame");


    //Publishers
    gazePublisher = nh_.advertise<geometry_msgs::Point>("fixation_point_out", 1);

    // Subscribers
    /* How do we read joint values on the real one? For now let's leave this commented. Rui and Plinio will teach me how to do this
     *
    neck_pan_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/neck_pan_position_controller/state", 10));
    neck_tilt_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/neck_tilt_position_controller/state", 10));
    eyes_tilt_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/eyes_tilt_position_controller/state", 10));
    version_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/version_position_controller/state", 10));
    vergence_sub=boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > (new message_filters::Subscriber<control_msgs::JointControllerState>(nh_, "/vizzy/vergence_position_controller/state", 10));
    */

    /*     fixation_point_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > (new message_filters::Subscriber<geometry_msgs::PointStamped>(nh_, "fixation_point", 10));

     //We will want to read joint states, so this remains here commented
    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
                                                                                                                          *neck_pan_sub,
                                                                                                                          *neck_tilt_sub,
                                                                                                                          *eyes_tilt_sub,
                                                                                                                          *version_sub,
                                                                                                                          *vergence_sub,
                                                                                                                        *fixation_point_sub));

    sync->registerCallback(boost::bind(&GazeReal::analysisCB, this, _1, _2, _3, _4, _5, _6));
*/

    //For now lets just do it for fixation_point_sub.

    fix_point_sub = nh_.subscribe("fixation_point", 1, &GazeReal::analysisCB, this);

    as_.registerGoalCallback(boost::bind(&Gaze::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Gaze::preemptCB, this));

    as_.start();

    ROS_INFO("Going to move head and eyes to home position.");
    moveHome();

}


// TODO
bool GazeReal::moveHome()
{

    //We don't have control of the joints yet. So we need to set a decent initial fixation point by hand.
    //Let's set the point in the base_link frame x=-20, y=0, z=0.959
    //-20 because base_link has the x axis reversed

    //Modo martelanço activado... lol
    geometry_msgs::Point goalToRobot;
    goalToRobot.x = -20;
    goalToRobot.y = 0;
    goalToRobot.z = 0.959;
    //Modo martelanço desactivado
    home_position_fixation_point.point=goalToRobot;
    home_position_fixation_point.header.frame_id=world_frame;
    home_position_fixation_point.header.stamp=ros::Time::now();
    gazePublisher.publish(goalToRobot);

}

bool GazeReal::moveCartesian()
{
    //One day we will probably control each joint value here. But now we will just leave the fixation point for the ros-yarp bridge
    //to process


    geometry_msgs::PointStamped goal_point;



    //Ros-yarp bridge receives the point in the waist frame, right? Let's get it in that frame doing another copy/paste from Rui's code

    while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(fixation_point_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
            tf_listener->transformPoint(fixation_point_frame, current_time, goal_msg->fixation_point, world_frame, goal_point);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }


    //Convert it to point, since ros-yarp wont receive a pointStamped
    geometry_msgs::Point goalToRobot;
    goalToRobot.x = goal_point.point.x;
    goalToRobot.y = goal_point.point.y;
    goalToRobot.z = goal_point.point.z;

    gazePublisher.publish(goalToRobot);

    return true;
}

void GazeReal::analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg)
{
    if(!active)
        return;

    //I need to know how the real robot gives us feedback regarding the current fixation point

    // Convert points to world frame
    geometry_msgs::PointStamped fixation_point_;
    geometry_msgs::PointStamped goal_point_;


    // Move home check joint state
    if(goal_msg->type==move_robot_msgs::GazeGoal::HOME)
    {
        while(nh_.ok())
        {
            try
            {
                ros::Time current_time = ros::Time::now();
                tf_listener->waitForTransform(world_frame, current_time, fixation_point_msg->header.frame_id, fixation_point_msg->header.stamp, world_frame, ros::Duration(10.0) );
                tf_listener->transformPoint(world_frame, current_time, *fixation_point_msg, world_frame, fixation_point_);
                tf_listener->waitForTransform(world_frame, current_time, home_position_fixation_point.header.frame_id, home_position_fixation_point.header.stamp, world_frame, ros::Duration(10.0) );
                tf_listener->transformPoint(world_frame, current_time, home_position_fixation_point, world_frame, goal_point_);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }
            break;
        }

        double error_x=fixation_point_.point.x-goal_point_.point.x;
        double error_y=fixation_point_.point.y-goal_point_.point.y;
        double error_z=fixation_point_.point.z-goal_point_.point.z;
        double error=sqrt(error_x*error_x+error_y*error_y+error_z*error_z);


        if(error < 0.001)
        {
            feedback_.state_reached=true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);

            ros::WallTime total_time = ros::WallTime::now();
            ROS_INFO_STREAM(action_name_.c_str()<<": Total time: " <<  (total_time - start_time).toSec());
            active=false;
        }
    }
    else
    {

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
                ROS_WARN("%s",ex.what());
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
    }
    as_.publishFeedback(feedback_);



}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    GazeReal gaze(ros::this_node::getName());
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

