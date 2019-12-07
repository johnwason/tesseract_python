%{
#include <tesseract_scene_graph/resource_locator.h>
%}

%feature("director") ResourceLocator;
%shared_ptr(tesseract_scene_graph::ResourceLocator)

namespace tesseract_scene_graph
{
class ResourceLocator
{
public:
  using Ptr = std::shared_ptr<ResourceLocator>;
  using ConstPtr = std::shared_ptr<const ResourceLocator>;

  virtual tesseract_common::Resource::Ptr locateResource(const std::string& url) = 0;
};

}  // namespace tesseract_scene_graph
