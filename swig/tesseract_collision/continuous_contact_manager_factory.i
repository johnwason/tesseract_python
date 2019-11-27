%{
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
%}

%include "continuous_contact_manager.i"

//%shared_ptr(ContinuousContactManagerFactory)

namespace tesseract_collision
{

class ContinuousContactManagerFactory
{
public:
  //using CreateMethod = std::function<ContinuousContactManager::Ptr()>;
  ContinuousContactManagerFactory();

  //bool registar(const std::string name, CreateMethod create_function);

  ContinuousContactManager::Ptr create(const std::string& name) const;

};
}  // namespace tesseract_collision