#ifndef GAZE_H
#define GAZE_H
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
// MoveIt!
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
#include <std_msgs/Float64.h>

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
    std::string eyes_center_frame;
    std::string ego_frame;
    moveit::core::RobotStatePtr kinematic_state;
    void publishFixationPoint(const move_robot_msgs::GazeGoalConstPtr &goal, bool valid);

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

    ros::Publisher fixation_point_marker_pub;
    ros::Publisher gaze_arrow_marker_pub;

    robot_model_loader::RobotModelLoader robot_model_loader;//("robot_description");
    moveit::core::RobotModelPtr robot_model;// = robot_model_loader.getModel();

    const moveit::core::JointModelGroup* head_joint_model_group;//kinematic_model->getJointModelGroup("head");
    moveit::planning_interface::MoveGroup* head_group;

    const moveit::core::JointModelGroup* eyes_joint_model_group;//kinematic_model->getJointModelGroup("head");
    moveit::planning_interface::MoveGroup* eyes_group;

    std::vector<double> head_joint_values;
    std::vector<std::string> head_joint_names;

    std::vector<double> eyes_joint_values;
    std::vector<std::string> eyes_joint_names;

public:
    double half_base_line;
    Gaze(const std::string & name);
    bool move(const geometry_msgs::PointStamped  &goal);
    void executeCB(const move_robot_msgs::GazeGoalConstPtr &goal);
};

#endif // GAZE_H
