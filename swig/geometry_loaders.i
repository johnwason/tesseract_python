%{
#include "geometry_loaders.h"
%}
%include <pybuffer.i>

%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);

%shared_ptr(BytesResource)
class BytesResource : public tesseract_common::Resource
{
public:
  BytesResource(const std::string& url, const uint8_t* bytes, size_t bytes_len);
  virtual bool isFile() override;
  virtual std::string getUrl() override;
  virtual std::string getFilePath() override;
  virtual std::vector<uint8_t> getResourceContents() override;
  virtual std::shared_ptr<std::istream> getResourceContentStream() override;
};

std::vector<tesseract_geometry::Mesh::Ptr> createMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);

std::vector<tesseract_geometry::SDFMesh::Ptr> createSDFMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);

std::vector<tesseract_geometry::ConvexMesh::Ptr> createConvexMeshFromBytes(const std::string& url, const uint8_t* bytes, size_t bytes_len, 
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false);
