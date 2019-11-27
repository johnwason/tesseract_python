%{
#include <tesseract/inverse_kinematics_manager.h>
%}

%shared_ptr(tesseract::InverseKinematicsManager)

namespace tesseract
{
class InverseKinematicsManager
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsManager>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsManager>;

  InverseKinematicsManager();
  virtual ~InverseKinematicsManager();

  bool registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::Ptr factory);

  void removeInvKinematicsFactory(const std::string& name);

  std::vector<std::string> getAvailableInvKinematicsSolvers() const;

  std::vector<std::string>
  getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const;

  tesseract_kinematics::InverseKinematicsFactory::ConstPtr getInvKinematicFactory(const std::string& name) const;

  bool addInvKinematicSolver(tesseract_kinematics::InverseKinematics::ConstPtr solver);

  void removeFwdKinematicSolver(const std::string& manipulator, const std::string& name);

  std::vector<std::string> getAvailableInvKinematicsManipulators() const;

  bool setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name);

  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;

  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator) const;

};
}  // namespace tesseract