%{
#include <tesseract_common/types.h>
%}

%define tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
//tesseract_aligned_vector(VectorVector3d, Eigen::Vector3d);
//TODO: Why is this not aligned??
%template(VectorVector3d) std::vector<Eigen::Vector3d>;
tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);

tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);

namespace tesseract_common
{
typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
//TODO: Whis is this not aligned??
typedef std::vector<Eigen::Vector3d> VectorVector3d;
typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> TransformMap;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;

struct JointTrajectory
{
  std::vector<std::string> joint_names;   /**< @brief The joint names */
  tesseract_common::TrajArray trajectory; /**< @brief The generated trajectory */
};

}