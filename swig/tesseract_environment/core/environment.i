%{
#include <tesseract_environment/core/environment.h>
%}

%include "state_solver.i"
%include "types.i"
%include "commands.i"

%shared_ptr(tesseract_environment::Environment)
%shared_ptr(tesseract_environment::StateSolver)

namespace tesseract_environment
{
class Environment
{
public:
  
  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;

  Environment();

  virtual ~Environment();

  virtual bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph) = 0;

  int getRevision() const { return revision_; }

  const Commands& getCommandHistory() const;

  virtual bool checkInitialized() const;

  virtual const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const;

  virtual void setName(const std::string& name);

  virtual const std::string& getName() const;

  virtual void setState(const std::unordered_map<std::string, double>& joints);
  virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  virtual EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const std::vector<double>& joint_values) const;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  virtual EnvState::ConstPtr getCurrentState() const;

  virtual bool addLink(tesseract_scene_graph::Link link);

  virtual bool addLink(tesseract_scene_graph::Link link, tesseract_scene_graph::Joint joint);

   virtual bool removeLink(const std::string& name);

  virtual bool moveLink(tesseract_scene_graph::Joint joint);

  virtual tesseract_scene_graph::Link::ConstPtr getLink(const std::string& name) const;

  virtual tesseract_scene_graph::Joint::ConstPtr getJoint(const std::string& name) const;

  virtual bool removeJoint(const std::string& name);

  virtual bool moveJoint(const std::string& joint_name, const std::string& parent_link);

  virtual bool changeJointOrigin(const std::string& joint_name, const Eigen::Isometry3d& new_origin);

  virtual void setLinkCollisionEnabled(const std::string& name, bool enabled);

  virtual bool getLinkCollisionEnabled(const std::string& name) const;

  virtual void setLinkVisibility(const std::string& name, bool visibility);

  virtual bool getLinkVisibility(const std::string& name) const;

  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  virtual void removeAllowedCollision(const std::string& link_name);

  virtual tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  virtual std::vector<std::string> getJointNames() const;

  virtual std::vector<std::string> getActiveJointNames() const;

  virtual Eigen::VectorXd getCurrentJointValues() const;

  virtual Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  virtual const std::string& getRootLinkName() const;

  virtual std::vector<std::string> getLinkNames() const;

  virtual std::vector<std::string> getActiveLinkNames() const;

  virtual tesseract_common::VectorIsometry3d getLinkTransforms() const;

  virtual const Eigen::Isometry3d& getLinkTransform(const std::string& link_name) const;

  virtual StateSolver::Ptr getStateSolver() const;

  virtual bool setActiveDiscreteContactManager(const std::string& name);

  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager() const;
  
  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager(const std::string& name) const;

  virtual bool setActiveContinuousContactManager(const std::string& name);

  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager() const;

  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager(const std::string& name) const;

  bool registerDiscreteContactManager(const std::string name,
                                      tesseract_collision::DiscreteContactManagerFactory::CreateMethod create_function);

  bool
  registerContinuousContactManager(const std::string name,
                                   tesseract_collision::ContinuousContactManagerFactory::CreateMethod create_function);

};
}  // namespace tesseract_environment
