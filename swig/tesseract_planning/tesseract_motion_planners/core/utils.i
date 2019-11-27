%{
#include <tesseract_motion_planners/core/utils.h>
%}

namespace tesseract_motion_planners
{
inline tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                                      const Eigen::Isometry3d& stop,
                                                      int steps);

inline std::vector<Waypoint::Ptr> interpolate(const Waypoint& start, const Waypoint& stop, int steps);

}  // namespace tesseract_motion_planners