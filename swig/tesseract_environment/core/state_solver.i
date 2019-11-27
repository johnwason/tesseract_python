%{
#include <tesseract_environment/core/state_solver.h>
%}

%shared_ptr(tesseract_environment::StateSolver)

namespace tesseract_environment
{
class StateSolver
{
public:
  using Ptr = std::shared_ptr<StateSolver>;
  using ConstPtr = std::shared_ptr<const StateSolver>;

  virtual ~StateSolver();

  virtual bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph) = 0;

    virtual void setState(const std::unordered_map<std::string, double>& joints) = 0;
  virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) = 0;
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) = 0;

  
  virtual EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const = 0;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const std::vector<double>& joint_values) const = 0;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  virtual EnvState::ConstPtr getCurrentState() const = 0;

  
  virtual Ptr clone() const = 0;

};
}  // namespace tesseract_environment