%{
#include <tesseract_environment/core/types.h>
%}

%shared_ptr(tesseract_environment::EnvState)
%shared_ptr(tesseract_environment::AdjacencyMapPair)
%shared_ptr(tesseract_environment::AdjacencyMap)

namespace tesseract_environment
{
enum class BodyType
{
  ROBOT_LINK = 0,
  ROBOT_ATTACHED = 1
};

struct EnvState
{
  using Ptr = std::shared_ptr<EnvState>;
  using ConstPtr = std::shared_ptr<const EnvState>;

  std::unordered_map<std::string, double> joints;
  tesseract_common::TransformMap transforms;
};

struct AdjacencyMapPair
{
  using Ptr = std::shared_ptr<AdjacencyMapPair>;
  using ConstPtr = std::shared_ptr<const AdjacencyMapPair>;

  std::string link_name;
  Eigen::Isometry3d transform;
};

class AdjacencyMap
{
public:
  using Ptr = std::shared_ptr<AdjacencyMap>;
  using ConstPtr = std::shared_ptr<const AdjacencyMap>;

  AdjacencyMap(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
               const std::vector<std::string>& active_links,
               const tesseract_common::TransformMap& state);

  virtual ~AdjacencyMap();

  const std::vector<std::string>& getActiveLinkNames() const;

  AdjacencyMapPair::ConstPtr getLinkMapping(const std::string& link_name) const;
};

}  // namespace tesseract_environment