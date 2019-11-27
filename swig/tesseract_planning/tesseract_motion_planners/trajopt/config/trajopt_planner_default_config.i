%{
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptPlannerDefaultConfig)

namespace tesseract_motion_planners
{
struct TrajOptPlannerDefaultConfig : public TrajOptPlannerConfig
{
  
  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const tesseract_common::VectorIsometry3d& tcp_);

  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const Eigen::Isometry3d& tcp_);

  virtual std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI() const;

  virtual bool generate() override;

  tesseract::Tesseract::ConstPtr tesseract;
  
  std::string manipulator;
  
  std::string link;

  sco::ModelType optimizer = sco::ModelType::AUTO_SOLVER;

  tesseract_common::VectorIsometry3d tcp;

  std::vector<Waypoint::Ptr> target_waypoints;

  trajopt::InitInfo::Type init_type;
  
  trajopt::TrajArray seed_trajectory;
  
  JointWaypoint::ConstPtr configuration;

  bool collision_check = true;
  
  bool collision_continuous = true;
  
  double collision_safety_margin = 0.025;
  
  bool smooth_velocities = true;
  
  Eigen::VectorXd velocity_coeff;
  
  bool smooth_accelerations = true;
  
  Eigen::VectorXd acceleration_coeff;
  
  bool smooth_jerks = true;
  
  Eigen::VectorXd jerk_coeff;

  // std::vector<std::pair<sco::VectorOfVector::func, sco::MatrixOfVector::func>> constraint_error_functions;
};

}  // namespace tesseract_motion_planners