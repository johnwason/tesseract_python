%{
#include <tesseract_geometry/impl/box.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Box)

namespace tesseract_geometry
{
class Box : public Geometry
{
public:
  using Ptr = std::shared_ptr<Box>;
  using ConstPtr = std::shared_ptr<const Box>;

  Box(double x, double y, double z);
  ~Box();

  double getX();
  double getY();
  double getZ();

};
}  // namespace tesseract_geometry
