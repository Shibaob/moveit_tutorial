/*LERPPlannerManager*/
/*通过manager找到规划器， 同时设置避障算法（FCL）， req发送到规划器，最后在做规划*/

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <class_loader/class_loader.hpp>
#include "lerp_interface/lerp_planning_context.h"

namespace lerp_interface
{
  // 继承父类
class LERPPlannerManager : public planning_interface::PlannerManager
{
public:
  LERPPlannerManager() : planning_interface::PlannerManager()
  {
  }

  //初始化运动规划器，该函数将会在构建函数之后执行，同时在运动规划调用前初始化完毕
  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override
  {
    for (const std::string& gpName : model->getJointModelGroupNames())
    {
	  //打印当前规划组和机器人模型的名称
      std::cout << "group name " << gpName << std::endl << "robot model  " << model->getName() << std::endl;
      
	  //创建运动规划插件的实例（lerp_planning_context.cpp）
	  planning_contexts_[gpName] =
          LERPPlanningContextPtr(new LERPPlanningContext("lerp_planning_context", gpName, model));
    }
    return true;
  }

  //该插件是否可以应答运动规划的请求（当前的插件接口是否能为上层做服务）
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
	//如果当前空闲即可应答
    return req.trajectory_constraints.constraints.empty();
  }

  //获取运动规划器的字符串描述
  std::string getDescription() const override
  {
    return "LERP";
  }

  //获取运动规划算法的名称 （一个规划器中的规划算法并不是唯一的）
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("lerp");
  }

  //构造规划器上下文，根据传入的场景信息和运动请求，应答运动规划的结果
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    //没有规划组
    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

	//没有场景信息
    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    //使用混合碰撞检测器创建规划场景
    planning_scene::PlanningScenePtr ps = planning_scene->diff();

    //将FCL算法设置为碰撞检测算法
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(), true);

    //检索和配置现有上下文 （通过manager调用规划其，打印信息）
    const LERPPlanningContextPtr& context = planning_contexts_.at(req.group_name);
    ROS_INFO_STREAM_NAMED("lerp_planner_manager", "===>>> context is made ");

	//设置规划场景，主要是碰撞检测算法
    context->setPlanningScene(ps);
	//设置运动规划的请求
    context->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    // 规划结束，控制器控制机械臂移动

    return context;
  }

protected:
  std::map<std::string, LERPPlanningContextPtr> planning_contexts_;
};

}  // namespace lerp_interface

// 将LERPPlannerManager类注册为一个插件（实现后的子类，继承的基类）
CLASS_LOADER_REGISTER_CLASS(lerp_interface::LERPPlannerManager, planning_interface::PlannerManager);
