/*自定义的class，把规划算法做封装*/
#pragma once

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPInterface);

class LERPInterface
{
public:
  LERPInterface(const ros::NodeHandle& nh = ros::NodeHandle("~"));
	
  // 求解器，输入的参数为规划场景，规划请求以及response
  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req, moveit_msgs::MotionPlanDetailedResponse& res);

protected:
  ros::NodeHandle nh_;
  std::string name_;
  int num_steps_;
  int dof_;

private:
  void interpolate(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& robot_state,
                   const robot_state::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                   const std::vector<double>& goal_joint_vals, trajectory_msgs::JointTrajectory& joint_trajectory);
};
}
