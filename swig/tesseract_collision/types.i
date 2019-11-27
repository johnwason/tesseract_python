%{
#include <tesseract_collision/core/types.h>
%}



//%tesseract_aligned_vector(ContactResultVector, tesseract_collision::ContactResult);
//%template(ContactResultMap) std::map<std::pair<std::string,std::string>,std::vector<tesseract::ContactResult , Eigen::aligned_allocator<tesseract::ContactResult >>,std::less<std::pair<std::string,std::string>>,Eigen::aligned_allocator<std::pair<const std::pair<std::string,std::string>,std::vector<tesseract::ContactResult , Eigen::aligned_allocator<tesseract::ContactResult >>>>>;

namespace tesseract_collision
{
using CollisionShapesConst = std::vector<tesseract_geometry::Geometry::ConstPtr>;
using CollisionShapeConstPtr = tesseract_geometry::Geometry::ConstPtr;
using CollisionShapePtr = tesseract_geometry::Geometry::Ptr;

enum class ContinouseCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

enum class ContactTestType
{
  FIRST = 0,
  CLOSEST = 1,
  ALL = 2,
  LIMITED = 3
};

struct ContactResult
{
  double distance;
  int type_id[2];
  std::string link_names[2];
  int shape_id[2];
  int subshape_id[2];
  Eigen::Vector3d nearest_points[2];
  Eigen::Vector3d normal;
  Eigen::Vector3d cc_nearest_points[2];
  double cc_time;
  ContinouseCollisionType cc_type;

  ContactResult();
  void clear();
};

using ContactResultVector = tesseract_common::AlignedVector<ContactResult>;
using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;
}

%inline
{
//std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v);
//TODO: this function has lousy performance
tesseract_collision::ContactResultVector flattenResults(tesseract_collision::ContactResultMap m)
{
    tesseract_collision::ContactResultVector v;
    tesseract_collision::flattenResults(std::move(m),v);
    return v;
}
}

namespace tesseract_collision
{
/*struct ContactTestData
{
  ContactTestData(const std::vector<std::string>& active,
                  const double& contact_distance,
                  const IsContactAllowedFn& fn,
                  const ContactTestType& type,
                  ContactResultMap& res);

  const std::vector<std::string>& active;
  const double& contact_distance;
  const IsContactAllowedFn& fn;
  const ContactTestType& type;
  
  // TODO: This can cause a lifecycle issue in Python!
  ContactResultMap& res;

  bool done;
};*/
}