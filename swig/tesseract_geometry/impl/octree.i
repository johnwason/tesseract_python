%{
#include <tesseract_geometry/impl/octree.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Octree)


namespace tesseract_geometry
{

%nodefaultctor Octree;
class Octree : public Geometry
{
public:
  using Ptr = std::shared_ptr<Octree>;
  using ConstPtr = std::shared_ptr<const Octree>;

  enum SubType
  {
    BOX,
    SPHERE_INSIDE,
    SPHERE_OUTSIDE
  };

  ~Octree() override = default;

  SubType getSubType() const { return sub_type_; }

  long calcNumSubShapes() const;

};
}