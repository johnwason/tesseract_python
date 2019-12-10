/**
 * @file cylinder.i
 * @brief SWIG interface file for tesseract_environment/impl/cylinder.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

%{
#include <tesseract_geometry/impl/cylinder.h>
%}

%include "geometry.i"

%shared_ptr(tesseract_geometry::Cylinder)

namespace tesseract_geometry
{
class Cylinder : public Geometry
{
public:
  using Ptr = std::shared_ptr<Cylinder>;
  using ConstPtr = std::shared_ptr<const Cylinder>;

  Cylinder(double r, double l);
  ~Cylinder() override;

  double getRadius() const;
  double getLength() const;
};
}  // namespace tesseract_geometry