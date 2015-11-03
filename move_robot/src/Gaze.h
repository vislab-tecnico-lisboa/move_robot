#ifndef GAZE_H
#define GAZE_H
// MoveIt!
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_robot_msgs/GazeAction.h>
#include <tf/transform_listener.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class Gaze
{
    double y_offset;
    double z_offset;

    std::string left_eye_frame;
    std::string right_eye_frame;
    std::string head_origin_frame;
    std::string eyes_center_frame;
    std::string neck_frame;
    std::string world_frame;

    void publishFixationPoint(const Eigen::Vector3d &goal, const std::string & frame_id, const bool valid);
    Eigen::Vector3d last_fixation_point;

     move_robot_msgs::GazeGoalConstPtr goal;
     ros::WallTime start_time;
protected:
    double distance_ego_eyes;
    planning_scene_monitor::CurrentStateMonitorPtr state_monitor;
    boost::shared_ptr<tf::TransformListener> tf_listener;

    ros::NodeHandle nh_;
    ros::NodeHandle private_node_handle;

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<move_robot_msgs::GazeAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    move_robot_msgs::GazeFeedback feedback_;
    move_robot_msgs::GazeResult result_;

    ros::Publisher fixation_point_goal_pub;

    moveit::core::RobotStatePtr kinematic_state;

    robot_model_loader::RobotModelLoader robot_model_loader;//("robot_description");
    moveit::core::RobotModelPtr robot_model;// = robot_model_loader.getModel();

    const moveit::core::JointModelGroup* oculocephalic_joint_model_group;//kinematic_model->getJointModelGroup("head");
    moveit::planning_interface::MoveGroup* oculocephalic_group;

    std::vector<double> oculocephalic_joint_values;
    std::vector<std::string> oculocephalic_joint_names;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, moveit_msgs::MoveGroupActionFeedback> MySyncPolicy;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > fixation_point_sub;
    boost::shared_ptr<message_filters::Subscriber<moveit_msgs::MoveGroupActionFeedback> > move_group_action_feedback_sub;

    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;
public:
    double half_base_line;
    Gaze(const std::string & name);
    bool move(const geometry_msgs::PointStamped  &goal);
    void goalCB();
    void preemptCB();
    void analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg, const moveit_msgs::MoveGroupActionFeedback::ConstPtr &move_group_action_feedback_msg);
};

#endif // GAZE_H
