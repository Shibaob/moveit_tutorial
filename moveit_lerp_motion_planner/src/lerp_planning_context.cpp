#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include "lerp_interface/lerp_planning_context.h"
#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
  LERPPlanningContext::LERPPlanningContext(const std::string &context_name, const std::string &group_name,
                                           const robot_model::RobotModelConstPtr &model)
      : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
  {
    // 修改指针指向的算法
    lerp_interface_ = LERPInterfacePtr(new LERPInterface());
  }

  //解决运动规划的求解问题，并且将详细的结果保存到应答数据中
  bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
  {
    moveit_msgs::MotionPlanDetailedResponse res_msg;

    //调用规划算法求解运动规划问题，返回是否求解成功
    bool lerp_solved = lerp_interface_->solve(planning_scene_, request_, res_msg);

    //如果求解成功，就将求解出的完整轨迹、处理时间等信息放入应答的数据结构中
    if (lerp_solved)
    {
      res.trajectory_.resize(1);
      res.trajectory_[0] =
          robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

      moveit::core::RobotState start_state(robot_model_);
      robot_state::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);

      res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
      res.description_.push_back("plan");
      res.processing_time_ = res_msg.processing_time;
      res.error_code_ = res_msg.error_code;

      return true;
    }

    // 求解失败
    res.error_code_ = res_msg.error_code;
    return false;
  };

  //解决运动规划的求解问题，并且将结果保存到应答数据中
  bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse &res)
  {
    planning_interface::MotionPlanDetailedResponse res_detailed;

    //调用规划算法求解运动规划问题，返回是否求解成功
    bool planning_success = solve(res_detailed);

    res.error_code_ = res_detailed.error_code_;

    //如果求解成功，应答中的轨迹只包含当前位姿，没有完整的轨迹
    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }

    return planning_success;
  }

  bool LERPPlanningContext::terminate()
  {
    // 中止，例如减速停止过程
    return true;
  }

  void LERPPlanningContext::clear()
  {
    // 规划器比较简单，没有什么需要清除的数据
  }

} // namespace lerp_interface
