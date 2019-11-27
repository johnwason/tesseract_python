%{
#include <tesseract_scene_graph/link.h>
%}

%include "joint.i"

%shared_ptr(tesseract_scene_graph::Material)
%shared_ptr(tesseract_scene_graph::Inertial)
%shared_ptr(tesseract_scene_graph::Visual)
%shared_ptr(tesseract_scene_graph::Collision)
%shared_ptr(tesseract_scene_graph::Link)
%shared_ptr(tesseract_geometry::Geometry)

%template(tesseract_scene_graph_VisualVector) std::vector<tesseract_scene_graph::Visual::Ptr>;
%template(tesseract_scene_graph_CollisionVector) std::vector<tesseract_scene_graph::Collision::Ptr>;

namespace tesseract_geometry
{
    class Geometry;
}

namespace tesseract_scene_graph
{
class Material
{
public:
 
  using Ptr = std::shared_ptr<Material>;
  using ConstPtr = std::shared_ptr<const Material>;

  Material(const std::string& name);

  const std::string& getName();

  std::string texture_filename;
  Eigen::Vector4d color;

  void clear();
};

class Inertial
{
public:

  using Ptr = std::shared_ptr<Inertial>;
  using ConstPtr = std::shared_ptr<const Inertial>;

  Inertial();
  Eigen::Isometry3d origin;
  double mass;
  double ixx, ixy, ixz, iyy, iyz, izz;

  void clear();
};

class Visual
{
public:
  
  using Ptr = std::shared_ptr<Visual>;
  using ConstPtr = std::shared_ptr<const Visual>;

  Visual();
  Eigen::Isometry3d origin;
  tesseract_geometry::Geometry::Ptr geometry;

  Material::Ptr material;

  void clear();
  std::string name;
};

class Collision
{
public:
  
  using Ptr = std::shared_ptr<Collision>;
  using ConstPtr = std::shared_ptr<const Collision>;

  Collision();
  Eigen::Isometry3d origin;
  tesseract_geometry::Geometry::Ptr geometry;

  void clear();

  std::string name;
};

class Link
{
public:
  using Ptr = std::shared_ptr<Link>;
  using ConstPtr = std::shared_ptr<const Link>;

  Link(std::string name);

  const std::string& getName() const;

  Inertial::Ptr inertial;

  std::vector<Visual::Ptr> visual;

  std::vector<Collision::Ptr> collision;

  void clear();
};

}  // namespace tesseract_scene_graph