%{
#include <tesseract_scene_graph/allowed_collision_matrix.h>
%}

%shared_ptr(tesseract_scene_graph::AllowedCollisionMatrix)

%inline %{
  struct AllowedCollisionEntry 
  {
    std::string link_name1;
    std::string link_name2;
    std::string reason;
  };
%}

%template(AllowedCollisionEntries) std::vector<AllowedCollisionEntry>;

namespace tesseract_scene_graph
{
class AllowedCollisionMatrix
{
public:
  
  using Ptr = std::shared_ptr<AllowedCollisionMatrix>;
  using ConstPtr = std::shared_ptr<const AllowedCollisionMatrix>;

  using AllowedCollisionEntries = std::unordered_map<LinkNamesPair, std::string, PairHash>;

  AllowedCollisionMatrix();
  virtual ~AllowedCollisionMatrix();

  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

%extend {
  
  std::vector<AllowedCollisionEntry> getAllAllowedCollisions()
  {
      std::vector<AllowedCollisionEntry> o;

      auto a = $self->getAllAllowedCollisions();
      for (const auto& a1 : a)
      {
        AllowedCollisionEntry o1;
        o1.link_name1 = a1.first.first;
        o1.link_name2 = a1.first.second;
        o1.reason = a1.second;
        o.push_back(o1);
      }
      return o;
  }  
}

  //const AllowedCollisionEntries getAllAllowedCollisions() const;

  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  virtual void removeAllowedCollision(const std::string& link_name);

  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;

  void clearAllowedCollisions();
};

}  // namespace tesseract_scene_graph
