/*PLANNING_CONTEXT*/
#ifndef LERP_PLANNING_CONTEXT_H
#define LERP_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>

#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPPlanningContext);

// 继承了Moveitplanning的PlanningContext基类来实现LERPPlanningContext这个类
class LERPPlanningContext : public planning_interface::PlanningContext
{
public:
  LERPPlanningContext(const std::string& name, const std::string& group, const robot_model::RobotModelConstPtr& model);
  ~LERPPlanningContext() override
  {
  }

  // planning_interface 中定义的纯虚文件
  //解决运动规划的求解问题，并且将结果保存到应答数据中
  bool solve(planning_interface::MotionPlanResponse& res) override;
  //解决运动规划的求解问题，并且将详细的结果保存到应答数据中
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  //终止运动规划的计算过程
  bool terminate() override;
  //清除运动规划器中的各种数据结构
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  LERPInterfacePtr lerp_interface_;
};

}  // namespace lerp_interface

#endif  // LERP_PLANNING_CONTEXT_H
