
//Automatic downcasting on return

%include "geometry.i"
%shared_factory(
    tesseract_geometry::Geometry,
    tesseract_geometry::Box,
	tesseract_geometry::Capsule,
	tesseract_geometry::Cone,
	tesseract_geometry::ConvexMesh,
	tesseract_geometry::Cylinder,
	tesseract_geometry::Mesh,
	tesseract_geometry::Octree,
	tesseract_geometry::Plane,
	tesseract_geometry::SDFMesh,
	tesseract_geometry::Sphere
)

%include "impl/box.i"
%include "impl/capsule.i"
%include "impl/cone.i"
%include "impl/convex_mesh.i"
%include "impl/cylinder.i"
%include "impl/mesh.i"
%include "impl/octree.i"
%include "impl/plane.i"
%include "impl/sdf_mesh.i"
%include "impl/sphere.i"
