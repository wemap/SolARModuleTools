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

class SOLAR_TOOLS_EXPORT_API SolAR3DTransform : public org::bcom::xpcf::ComponentBase,
        public api::geom::I3DTransform {
public:

    SolAR3DTransform();
   ~SolAR3DTransform() = default;

    /// @brief This method applies a transformation (4x4 float matrix) to a set of 3D points
    /// @param[in] inputPoints the set of 3D points to transform
    /// @param[in] transformation the 3D transformation to apply (a 4x4 float matrix)
    /// @param[out] outputPoints the resulting set of 3D points after 3D transformation
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode transform(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation, std::vector<SRef<Point3Df>> & outputPoints) const override;

    /// @brief This method applies a transformation (4x4 float matrix) to a point cloud
    /// @param[in] inputPointCloud the point cloud to transform
    /// @param[in] transformation the 3D transformation to apply (a 4x4 float matrix)
    /// @param[out] outputPointCLoud the resulting point cloud after 3D transformation
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode transform(const SRef<PointCloud> inputPointCloud, const Transform3Df transformation, SRef<PointCloud>& outputPointCloud) const override;

    /// @brief This method applies a transformation (4x4 float matrix) to a set of 3D points
    /// @param[in] inputPoints the set of 3D points to transform
    /// @param[in] transformation the 3D transformation to apply (a 4x4 float matrix)
    /// @param[out] outputPoints the resulting set of 3D points after 3D transformation
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode transformInPlace(std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation) const override;

    /// @brief This method applies a transformation (4x4 float matrix) to a point cloud
    /// @param[in] inputPointCloud the point cloud to transform
    /// @param[in] transformation the 3D transformation to apply (a 4x4 float matrix)
    /// @param[out] outputPointCLoud the resulting point cloud after 3D transformation
    /// @return FrameworkReturnCode::_SUCCESS_ if 3D transformation succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode transformInPlace(SRef<PointCloud> inputPointCloud, const Transform3Df transformation) const override;


    void unloadComponent () override final;


 private:


};

}
}
}

#endif // SOLAR3DTRANSFORM_H
