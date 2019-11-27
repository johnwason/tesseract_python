%{
#include <tesseract_geometry/impl/plane.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Plane)

namespace tesseract_geometry
{
class Plane : public Geometry
{
public:
  using Ptr = std::shared_ptr<Plane>;
  using ConstPtr = std::shared_ptr<const Plane>;

  Plane(double a, double b, double c, double d);
  ~Plane() override;

  double getA() const;
  double getB() const;
  double getC() const;
  double getD() const;

};
}  // namespace tesseract_geometry
