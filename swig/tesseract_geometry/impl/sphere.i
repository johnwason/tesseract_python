%{
#include <tesseract_geometry/impl/sphere.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Sphere)

namespace tesseract_geometry
{
class Sphere : public Geometry
{
public:
  using Ptr = std::shared_ptr<Sphere>;
  using ConstPtr = std::shared_ptr<const Sphere>;

  explicit Sphere(double r);
  ~Sphere() override;

  double getRadius() const;

  Geometry::Ptr clone() const override;

};
}  // namespace tesseract_geometry
