%{
#include <tesseract_geometry/geometry.h>
%}

%shared_ptr(tesseract_geometry::Geometry)

namespace tesseract_geometry
{
enum GeometryType
{
  SPHERE,
  CYLINDER,
  CAPSULE,
  CONE,
  BOX,
  PLANE,
  MESH,
  CONVEX_MESH,
  SDF_MESH,
  OCTREE
};

%nodefaultctor Geometry;
class Geometry
{
public:
  using Ptr = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<const Geometry>;

  virtual ~Geometry();

  virtual Geometry::Ptr clone();

  GeometryType getType() const { return type_; }

};

using Geometrys = std::vector<Geometry::Ptr>;
using GeometrysConst = std::vector<Geometry::ConstPtr>;
}  // namespace tesseract_geometry

%template(tesseract_geometry_Geometrys) std::vector<std::shared_ptr<tesseract_geometry::Geometry> >;
%template(tesseract_geometry_GeometrysConst) std::vector<std::shared_ptr<const tesseract_geometry::Geometry> >;


