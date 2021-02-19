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

   ~SolAR3DTransform() override;

    /// @brief This method applies a 3D transform to a set of 3D points
    /// @param[in] inputPoints The 3D points on which the 3D transform will be applied.
    /// @param[in] transformation The 3D transform to apply to the set of 3D points.
    /// @param[out] outputPoints The resulting 3D points after application of the 3D transform.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode transform(const std::vector<datastructure::Point3Df> & inputPoints, const datastructure::Transform3Df & transformation, std::vector<datastructure::Point3Df> & outputPoints) override;

	/// @brief This method applies a transformation (4x4 float matrix) to a map including point cloud and keyframes	
	/// @param[in] transformation: transformation the 3D transformation to apply (a 4x4 float matrix)
	/// @param[in,out] map: the map to apply the transformation
	/// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode transform(const datastructure::Transform3Df & transformation, SRef<api::solver::map::IMapper> map) override;

	/// @brief This method applies a transformation (4x4 float matrix) to a point cloud
	/// @param[in] inputPointCloud the point cloud to transform
	/// @param[in] transformation the 3D transformation to apply (a 4x4 float matrix)
	/// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode transformInPlace(SRef<datastructure::PointCloud> inputPointCloud, const datastructure::Transform3Df transformation) const override;

    void unloadComponent () override final;


 private:


};

}
}
}

#endif // SOLAR3DTRANSFORM_H
