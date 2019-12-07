%{
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptMotionPlannerStatusCategory)

namespace tesseract_motion_planners
{
class TrajOptMotionPlannerStatusCategory;

class TrajOptMotionPlanner : public MotionPlanner
{
public:
  
  TrajOptMotionPlanner(std::string name = "TRAJOPT");

  ~TrajOptMotionPlanner() {}

  //bool setConfiguration(const TrajOptPlannerConfig::Ptr config);
  %extend {
    void setConfiguration(const TrajOptPlannerConfig::Ptr config)
	{
	  if (!$self->setConfiguration(config))
      {
	     throw std::runtime_error("setConfiguration failed");
      }
	}
  }

  %rename(_solve) solve;
  tesseract_common::StatusCode solve(PlannerResponse& response, const bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  tesseract_common::StatusCode isConfigured() const override;
};

class TrajOptMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    IsNotConfigured = -1,
    FailedToParseConfig = -2,
    FailedToFindValidSolution = -3,
    FoundValidSolutionInCollision = -4
  };
};

}  // namespace tesseract_motion_planners