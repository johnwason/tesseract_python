%{
#include <tesseract_collision/core/discrete_contact_manager.h>
%}

%include "types.i"

%shared_ptr(tesseract_collision::DiscreteContactManager)

namespace tesseract_collision
{
class DiscreteContactManager
{
public:
  
  using Ptr = std::shared_ptr<DiscreteContactManager>;
  using ConstPtr = std::shared_ptr<const DiscreteContactManager>;

  virtual ~DiscreteContactManager();

  virtual std::shared_ptr<DiscreteContactManager> clone() const = 0;

  virtual bool addCollisionObject(const std::string& name,
                                  const int& mask_id,
                                  const CollisionShapesConst& shapes,
                                  const tesseract_common::VectorIsometry3d& shape_poses,
                                  bool enabled = true) = 0;

  virtual const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const = 0;

  virtual const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const = 0;

  virtual bool hasCollisionObject(const std::string& name) const = 0;

  
  virtual bool removeCollisionObject(const std::string& name) = 0;

  virtual bool enableCollisionObject(const std::string& name) = 0;

  virtual bool disableCollisionObject(const std::string& name) = 0;

  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract_common::VectorIsometry3d& poses) = 0;

  virtual void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) = 0;

  virtual void setActiveCollisionObjects(const std::vector<std::string>& names) = 0;

  virtual const std::vector<std::string>& getActiveCollisionObjects() const = 0;

  virtual void setContactDistanceThreshold(double contact_distance) = 0;

  virtual double getContactDistanceThreshold() const = 0;

  // TODO: IsContactAllowedFn
  //virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

  //virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;

  //virtual void contactTest(ContactResultMap& collisions, const ContactTestType& type) = 0;
  %extend {
    tesseract_collision::ContactResultMap contactTest(const tesseract_collision::ContactTestType& type)
    {
        tesseract_collision::ContactResultMap contacts;
        $self->contactTest(contacts, type);
        return contacts;
    }
  }
};

}  // namespace tesseract_collision
