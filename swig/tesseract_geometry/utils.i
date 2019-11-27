%include "geometry.i"

%{
#include <tesseract_geometry/utils.h>
%}

namespace tesseract_geometry
{
bool isIdentical(const Geometry& geom1, const Geometry& geom2);
}