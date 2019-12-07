%{
#include <tesseract_geometry/impl/sdf_mesh.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::SDFMesh)

%template(tesseract_geometry_SDFMeshVector) std::vector<tesseract_geometry::SDFMesh::Ptr>;

namespace tesseract_geometry
{
class SDFMesh : public Geometry
{
public:
  
  using Ptr = std::shared_ptr<SDFMesh>;
  using ConstPtr = std::shared_ptr<const SDFMesh>;

%extend {

  SDFMesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::SDFMesh(vertices1, faces1, resource, scale);
  }

  SDFMesh(const tesseract_common::VectorVector3d& vertices,
             const Eigen::VectorXi& faces,
             int face_count,
             tesseract_common::Resource::Ptr resource = nullptr,
             Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
      std::shared_ptr<const tesseract_common::VectorVector3d> vertices1 = std::make_shared<const tesseract_common::VectorVector3d>(vertices);
      std::shared_ptr<const Eigen::VectorXi> faces1 = std::make_shared<const Eigen::VectorXi>(faces);
      return new tesseract_geometry::SDFMesh(vertices1, faces1, face_count, resource, scale);
  }
}
  ~SDFMesh() override;

%extend {

  tesseract_common::VectorVector3d getVertices()
  {
    return *$self->getVertices();
  }

  Eigen::VectorXi getTriangles()
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