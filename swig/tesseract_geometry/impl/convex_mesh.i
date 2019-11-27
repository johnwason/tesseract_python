%{
#include <tesseract_geometry/impl/convex_mesh.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::ConvexMesh)

%template(tesseract_geometry_ConvexMeshVector) std::vector<tesseract_geometry::ConvexMesh::Ptr>;

namespace tesseract_geometry
{
class ConvexMesh : public Geometry
{
public:
  
  using Ptr = std::shared_ptr<ConvexMesh>;
  using ConstPtr = std::shared_ptr<const ConvexMesh>;

%extend {

  ConvexMesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::ConvexMesh(vertices1, faces1, resource, scale);
  }

  ConvexMesh(const const tesseract_common::VectorVector3d& vertices,
             const const Eigen::VectorXi& faces,
             int face_count,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::ConvexMesh(vertices1, faces1, face_count, resource, scale);
  }
}
  ~ConvexMesh() override;

%extend {

  const tesseract_common::VectorVector3d getVertices()
  {
    return *$self->getVertices();
  }

  const Eigen::VectorXi getFaces()
  {
    return *$self->getFaces();
  }
}

  int getVerticeCount() const;
  int getFaceCount() const;

  const tesseract_common::Resource::Ptr getResource();

  const Eigen::Vector3d& getScale() const { return scale_; }
};
}  // namespace tesseract_geometry
