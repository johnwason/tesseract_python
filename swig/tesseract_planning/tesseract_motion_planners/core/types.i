%{
#include <tesseract_motion_planners/core/types.h>
%}

%template(WaypointVector) std::vector<tesseract_motion_planners::Waypoint::Ptr>;

namespace tesseract_motion_planners
{
struct PlannerRequest
{
  std::string name;
  tesseract::Tesseract::ConstPtr tesseract;
  tesseract_environment::EnvState::ConstPtr start_state;
  std::string config;
  std::string config_format;
};

struct PlannerResponse
{
  tesseract_common::JointTrajectory joint_trajectory;
  tesseract_common::StatusCode status;
  std::vector<Waypoint::Ptr> succeeded_waypoints;
  std::vector<Waypoint::Ptr> failed_waypoints;
};
}  // namespace tesseract_motion_planners
