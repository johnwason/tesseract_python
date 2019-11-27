%{
#include <tesseract_common/resource.h>
%}

%shared_ptr(tesseract_common::Resource)

%template(vector_uint8) std::vector<uint8_t>;

namespace tesseract_common
{
class Resource
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  virtual bool isFile() = 0;
  virtual std::string getUrl() = 0;
  virtual std::string getFilePath() = 0;
  //TODO: Typemap to bytes instead of vector
  virtual std::vector<uint8_t> getResourceContents() = 0;  
};

}  // namespace tesseract_common