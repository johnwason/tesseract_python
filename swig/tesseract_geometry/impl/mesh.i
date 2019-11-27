%{
#include <tesseract_geometry/impl/mesh.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Mesh)

%template(tesseract_geometry_MeshVector) std::vector<tesseract_geometry::Mesh::Ptr>;

namespace tesseract_geometry
{
class Mesh : public Geometry
{
public:
  
  using Ptr = std::shared_ptr<Mesh>;
  using ConstPtr = std::shared_ptr<const Mesh>;

%extend {

  Mesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::Mesh(vertices1, faces1, resource, scale);
  }

  Mesh(const const tesseract_common::VectorVector3d& vertices,
             const const Eigen::VectorXi& faces,
             int face_count,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::Mesh(vertices1, faces1, face_count, resource, scale);
  }
}
  ~Mesh() override;

%extend {

  const tesseract_common::VectorVector3d getVertices()
  {
    return *$self->getVertices();
  }

  const Eigen::VectorXi getTriangles()
  {
    return *$self->getTriangles();
  }
}

  int getVerticeCount() const;
  int getTriangleCount() const;

  const tesseract_common::Resource::Ptr getResource();

  const Eigen::Vector3d& getScale() const { return scale_; }
};
}  // namespace tesseract_geometry
