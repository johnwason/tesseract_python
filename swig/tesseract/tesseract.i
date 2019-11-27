%{
#include <tesseract/tesseract.h>
%}

%shared_ptr(tesseract::Tesseract)

namespace tesseract
{
class Tesseract
{
public:
  using Ptr = std::shared_ptr<Tesseract>;
  using ConstPtr = std::shared_ptr<const Tesseract>;

  Tesseract();
  virtual ~Tesseract() = default;

  bool isInitialized() const;

  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph);
  //bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph, tesseract_scene_graph::SRDFModel::ConstPtr srdf_model);
  bool init(const std::string& urdf_string, tesseract_scene_graph::ResourceLocator::Ptr locator);
  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            tesseract_scene_graph::ResourceLocator::Ptr locator);
  //bool init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocator::Ptr locator);
  //bool init(const boost::filesystem::path& urdf_path,
  //          const boost::filesystem::path& srdf_path,
  //          tesseract_scene_graph::ResourceLocator::Ptr locator);

  //const tesseract_scene_graph::SRDFModel::ConstPtr& getSRDFModel() const;

  const tesseract_environment::Environment::Ptr getEnvironment();
  const tesseract_environment::Environment::ConstPtr getEnvironmentConst() const;

  const ForwardKinematicsManager::Ptr getFwdKinematicsManager();
  const ForwardKinematicsManager::ConstPtr getFwdKinematicsManagerConst() const;

  const InverseKinematicsManager::Ptr getInvKinematicsManager();
  const InverseKinematicsManager::ConstPtr getInvKinematicsManagerConst() const;

};

}  // namespace tesseract