%{
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
%}

%shared_ptr(tesseract_kinematics::InverseKinematicsFactory)

namespace tesseract_kinematics
{
enum class InverseKinematicsFactoryType
{
  CHAIN = 0,
  TREE = 1,
  GRAPH = 2
};

class InverseKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsFactory>;

  virtual ~InverseKinematicsFactory() = default;

  virtual const std::string& getName() const = 0;

  virtual InverseKinematicsFactoryType getType() const = 0;

  virtual InverseKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                        const std::string& base_link,
                                        const std::string& tip_link,
                                        const std::string name) const;

  virtual InverseKinematics::Ptr
  create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
         const std::vector<std::string>& joint_names,
         const std::string name,
         std::unordered_map<std::string, double> start_state = std::unordered_map<std::string, double>()) const;
};

}  // namespace tesseract_kinematics
