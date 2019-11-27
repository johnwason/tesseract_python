%{
#include <tesseract_geometry/impl/cylinder.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Cylinder)

namespace tesseract_geometry
{
class Cylinder : public Geometry
{
public:
  using Ptr = std::shared_ptr<Cylinder>;
  using ConstPtr = std::shared_ptr<const Cylinder>;

  Cylinder(double r, double l);
  ~Cylinder() override;

  double getRadius() const;
  double getLength() const;
};
}  // namespace tesseract_geometry