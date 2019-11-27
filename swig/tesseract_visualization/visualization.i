%{
#include <tesseract_visualization/visualization.h>
%}

%shared_ptr(tesseract_visualization::Visualization)

namespace tesseract_visualization
{
class Visualization
{
public:
  using Ptr = std::shared_ptr<Visualization>;
  using ConstPtr = std::shared_ptr<const Visualization>;

  virtual ~Visualization();
  virtual void plotTrajectory(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj) = 0;

  virtual void plotContactResults(const std::vector<std::string>& link_names,
                                  const tesseract_collision::ContactResultVector& dist_results,
                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances) = 0;

  virtual void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
                         double scale) = 0;

  virtual void plotAxis(const Eigen::Isometry3d& axis, double scale) = 0;

  virtual void clear() = 0;

  virtual void waitForInput() = 0;
};

}  // namespace tesseract_visualization
