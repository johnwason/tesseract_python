%{
#include <tesseract_geometry/impl/cone.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Cone)

namespace tesseract_geometry
{
class Cone : public Geometry
{
public:
  using Ptr = std::shared_ptr<Cone>;
  using ConstPtr = std::shared_ptr<const Cone>;

  Cone(double r, double l);
  ~Cone() override;

  double getRadius() const;
  double getLength() const;

  Geometry::Ptr clone() const override;
};

}  // namespace tesseract_geometry
