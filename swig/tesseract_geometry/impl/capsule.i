%{
#include <tesseract_geometry/impl/capsule.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Capsule)

namespace tesseract_geometry
{
class Capsule : public Geometry
{
public:
  using Ptr = std::shared_ptr<Capsule>;
  using ConstPtr = std::shared_ptr<const Capsule>;

  Capsule(double r, double l);
  ~Capsule() override;

  double getRadius() const;
  double getLength() const;
};
}  // namespace tesseract_geometry
