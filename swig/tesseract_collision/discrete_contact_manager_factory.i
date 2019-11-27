%{
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
%}

%include "discrete_contact_manager.i"

//%shared_ptr(DiscreteContactManagerFactory)

namespace tesseract_collision
{

class DiscreteContactManagerFactory
{
public:
  //using CreateMethod = std::function<DiscreteContactManager::Ptr()>;
  DiscreteContactManagerFactory();

  //bool registar(const std::string name, CreateMethod create_function);

  DiscreteContactManager::Ptr create(const std::string& name) const;

};
}  // namespace tesseract_collision