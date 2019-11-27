%{
#include <tesseract_motion_planners/core/waypoint.h>
%}

%shared_ptr(tesseract_motion_planners::Waypoint)
%shared_ptr(tesseract_motion_planners::JointWaypoint)
%shared_ptr(tesseract_motion_planners::CartesianWaypoint)
%shared_ptr(tesseract_motion_planners::JointTolerancedWaypoint)

namespace tesseract_motion_planners
{
enum class WaypointType
{
  JOINT_WAYPOINT,
  JOINT_TOLERANCED_WAYPOINT,
  CARTESIAN_WAYPOINT
};

inline bool isCartesianWaypointType(WaypointType type);

inline bool isJointWaypointType(WaypointType type);

class Waypoint
{
public:
  
  using Ptr = std::shared_ptr<Waypoint>;
  using ConstPtr = std::shared_ptr<const Waypoint>;

  Waypoint(WaypointType type);
  virtual ~Waypoint();
  
  WaypointType getType();

  virtual bool isCritical() const;
  
  virtual void setIsCritical(bool is_critical);

  virtual bool setCoefficients(Eigen::VectorXd coefficients);

  virtual const Eigen::VectorXd getCoefficients() const;
};

class JointWaypoint : public Waypoint
{
public:
  
  using Ptr = std::shared_ptr<JointWaypoint>;
  using ConstPtr = std::shared_ptr<const JointWaypoint>;

  JointWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names);

  const Eigen::VectorXd getPositions();

  Eigen::VectorXd getPositions(const std::vector<std::string>& joint_names) const;

  const Eigen::VectorXd getCoefficients() const override;

  Eigen::VectorXd getCoefficients(const std::vector<std::string>& joint_names) const;

  const std::vector<std::string> getNames() const;

  bool compare(const std::vector<std::string>& joint_names) const;

};

class CartesianWaypoint : public Waypoint
{
public:
  
  using Ptr = std::shared_ptr<CartesianWaypoint>;
  using ConstPtr = std::shared_ptr<const CartesianWaypoint>;

  CartesianWaypoint(Eigen::Isometry3d cartesian_position, std::string parent_link = "");

  const Eigen::Isometry3d getTransform();

  Eigen::Vector3d getPosition() const;
  
  Eigen::Vector4d getOrientation() const;

  std::string getParentLinkName() const;
};

class JointTolerancedWaypoint : public JointWaypoint
{
public:
  using Ptr = std::shared_ptr<JointTolerancedWaypoint>;
  using ConstPtr = std::shared_ptr<const JointTolerancedWaypoint>;

  JointTolerancedWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names);

  bool setUpperTolerance(Eigen::VectorXd upper_tolerance);

  const Eigen::VectorXd getUpperTolerance() const;

  Eigen::VectorXd getUpperTolerance(const std::vector<std::string>& joint_names) const;

  bool setLowerTolerance(Eigen::VectorXd lower_tolerance);

  const Eigen::VectorXd getLowerTolerance() const;

   Eigen::VectorXd getLowerTolerance(const std::vector<std::string>& joint_names) const;
};

}  // namespace tesseract_motion_planners