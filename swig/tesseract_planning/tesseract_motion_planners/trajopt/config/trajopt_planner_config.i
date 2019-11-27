%{
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptPlannerConfig)

namespace tesseract_motion_planners
{
struct TrajOptPlannerConfig
{
  using Ptr = std::shared_ptr<TrajOptPlannerConfig>;

  explicit TrajOptPlannerConfig() = default;
  explicit TrajOptPlannerConfig(trajopt::TrajOptProb::Ptr problem);

  virtual ~TrajOptPlannerConfig() = default;

  virtual bool generate();

  //TODO: ?
  //sco::BasicTrustRegionSQPParameters params;

  //TODO: ?
  //std::vector<sco::Optimizer::Callback> callbacks;
  
  trajopt::TrajOptProb::Ptr prob;
};

}  // namespace tesseract_motion_planners