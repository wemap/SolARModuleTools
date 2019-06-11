/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLAR3DTRANSFORM_H
#define SOLAR3DTRANSFORM_H

#include "api/geom/I3DTransform.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolAR3DTransform
 * @brief <B>Applies a 3D Transform to a set of 3D points.</B>
 * <TT>UUID: f05dd955-33bd-4d52-8717-93ad298ed3e3</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolAR3DTransform : public org::bcom::xpcf::ComponentBase,
        public api::geom::I3DTransform {
public:

    SolAR3DTransform();
   ~SolAR3DTransform();

    /// @brief This method applies a 3D transform to a set of 3D points
    /// @param[in] inputPoints The 3D points on which the 3D transform will be applied.
    /// @param[in] transformation The 3D transform to apply to the set of 3D points.
    /// @param[out] outputPoints The resulting 3D points after application of the 3D transform.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode transform(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation, std::vector<SRef<Point3Df>> & outputPoints);

    void unloadComponent () override final;


 private:


};

}
}
}

#endif // SOLAR3DTRANSFORM_H
