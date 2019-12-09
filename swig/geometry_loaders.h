#include <tesseract_geometry/mesh_parser.h>

#pragma once

class BytesResource : public tesseract_common::Resource
{
public:
  BytesResource(const std::string& url, const uint8_t* bytes, size_t bytes_len)
  {
    url_ = url;
    bytes_ = std::vector<uint8_t>(bytes, bytes + bytes_len);
  }
  virtual bool isFile() override
  {
    return false;
  }
  virtual std::string getUrl() override
  {
    return url_;
  }
  virtual std::string getFilePath() override
  {
    return "";
  }
  virtual std::vector<uint8_t> getResourceContents() override
  {
    return bytes_;
  }
  virtual std::shared_ptr<std::istream> getResourceContentStream() override
  {
    std::shared_ptr<std::stringstream> o = std::make_shared<std::stringstream>();
    o->write((const char*)&bytes_.at(0), bytes_.size());
    o->seekg(0, o->beg);
    return o;
  }

protected:
  std::string url_;
  std::vector<uint8_t> bytes_;
};

std::vector<tesseract_geometry::Mesh::Ptr> createMeshFromBytes(const std::string& url, const uint8_t* bytes,
                                                               size_t bytes_len,
                                                               Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                               bool triangulate = false, bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource = std::make_shared<BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(resource, scale, triangulate, flatten);
}

std::vector<tesseract_geometry::SDFMesh::Ptr> createSDFMeshFromBytes(const std::string& url, const uint8_t* bytes,
                                                                     size_t bytes_len,
                                                                     Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                                     bool triangulate = false, bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource = std::make_shared<BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::SDFMesh>(resource, scale, triangulate, flatten);
}

std::vector<tesseract_geometry::ConvexMesh::Ptr> createConvexMeshFromBytes(
    const std::string& url, const uint8_t* bytes, size_t bytes_len, Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
    bool triangulate = false, bool flatten = false)
{
  std::shared_ptr<tesseract_common::Resource> resource = std::make_shared<BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>(resource, scale, triangulate,
                                                                                    flatten);
}
