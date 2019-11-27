%{
#include <tesseract/forward_kinematics_manager.h>
%}

%shared_ptr(tesseract::ForwardKinematicsManager)

namespace tesseract
{
class ForwardKinematicsManager
{
public:
  using Ptr = std::shared_ptr<ForwardKinematicsManager>;
  using ConstPtr = std::shared_ptr<const ForwardKinematicsManager>;

  ForwardKinematicsManager();
  virtual ~ForwardKinematicsManager();

  bool registerFwdKinematicsFactory(tesseract_kinematics::ForwardKinematicsFactory::Ptr factory);
  
  void removeFwdKinematicsFactory(const std::string& name);

  std::vector<std::string> getAvailableFwdKinematicsSolvers() const;
  
  std::vector<std::string>
  getAvailableFwdKinematicsSolvers(tesseract_kinematics::ForwardKinematicsFactoryType type) const;
  
  tesseract_kinematics::ForwardKinematicsFactory::ConstPtr getFwdKinematicFactory(const std::string& name) const;
  
  bool addFwdKinematicSolver(tesseract_kinematics::ForwardKinematics::ConstPtr solver);
  
  void removeFwdKinematicSolver(const std::string& manipulator, const std::string& name);
  
  std::vector<std::string> getAvailableFwdKinematicsManipulators() const;
  
  bool setDefaultFwdKinematicSolver(const std::string& manipulator, const std::string& name);
    
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;
  
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator) const;
  
};
}  // namespace tesseract