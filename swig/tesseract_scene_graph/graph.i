%{
#include <tesseract_scene_graph/graph.h>
%}

%include "joint.i"
%include "link.i"
%include "allowed_collision_matrix.i"

%shared_ptr(tesseract_scene_graph::SceneGraph)

%template(tesseract_scene_graph_LinkVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link> >;
%template(tesseract_scene_graph_JointVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint> >;

%template(tesseract_scene_graph_LinkConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Link const> >;
%template(tesseract_scene_graph_JointConstVector) std::vector<std::shared_ptr<tesseract_scene_graph::Joint const> >;

namespace tesseract_scene_graph
{
class SceneGraph
{
public:

using Ptr = std::shared_ptr<SceneGraph>;
using ConstPtr = std::shared_ptr<const SceneGraph>;

SceneGraph();

void setName(const std::string& name);

const std::string& getName() const;

bool setRoot(const std::string& name);

const std::string& getRoot() const;

bool addLink(Link link);

Link::ConstPtr getLink(const std::string& name) const;

std::vector<Link::ConstPtr> getLinks() const;

bool removeLink(const std::string& name);

void setLinkVisibility(const std::string& name, bool visibility);

bool getLinkVisibility(const std::string& name) const;

void setLinkCollisionEnabled(const std::string& name, bool enabled);

bool getLinkCollisionEnabled(const std::string& name) const;

bool addJoint(Joint joint);

Joint::ConstPtr getJoint(const std::string& name) const;

bool removeJoint(const std::string& name);

bool moveJoint(const std::string& name, const std::string& parent_link);

std::vector<Joint::ConstPtr> getJoints() const;

bool changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin);

void addAllowedCollision(const std::string& link_name1, const std::string& link_name2, const std::string& reason);

void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

void removeAllowedCollision(const std::string& link_name);

void clearAllowedCollisions();

bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;

AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

Link::ConstPtr getSourceLink(const std::string& joint_name) const;

Link::ConstPtr getTargetLink(const std::string& joint_name) const;

std::vector<Joint::ConstPtr> getInboundJoints(const std::string& link_name) const;

std::vector<Joint::ConstPtr> getOutboundJoints(const std::string& link_name) const;

bool isAcyclic() const;

bool isTree() const;

std::vector<std::string> getAdjacentLinkNames(const std::string& name) const;

std::vector<std::string> getInvAdjacentLinkNames(const std::string& name) const;

std::vector<std::string> getLinkChildrenNames(const std::string& name) const;

std::vector<std::string> getJointChildrenNames(const std::string& name) const;

void saveDOT(std::string path) const;

// TODO: Boost Graph functions?
// Path getShortestPath(const std::string& root, const std::string& tip);
// Vertex getVertex(const std::string& name) const;
// Edge getEdge(const std::string& name) const;
};

} // namespace tesseract_scene_graph