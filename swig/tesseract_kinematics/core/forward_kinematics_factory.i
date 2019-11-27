%{
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
%}

%shared_ptr(tesseract_kinematics::ForwardKinematicsFactory)

namespace tesseract_kinematics
{
enum class ForwardKinematicsFactoryType
{
  CHAIN = 0,
  TREE = 1,
  GRAPH = 2
};

class ForwardKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<ForwardKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const ForwardKinematicsFactory>;

  virtual ~ForwardKinematicsFactory();

  virtual const std::string& getName() const = 0;

  virtual ForwardKinematicsFactoryType getType() const = 0;

  virtual ForwardKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                        const std::string& base_link,
                                        const std::string& tip_link,
                                        const std::string name) const;

  virtual ForwardKinematics::Ptr
  create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
         const std::vector<std::string>& joint_names,
         const std::string name,
         std::unordered_map<std::string, double> start_state = std::unordered_map<std::string, double>()) const;
};

}  // namespace tesseract_kinematics