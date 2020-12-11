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

#ifndef SOLAR2DTRANSFORM_H
#define SOLAR2DTRANSFORM_H

#include "api/geom/I2DTransform.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolAR2DTransform
 * @brief <B>Applies a 2D Transform to a set of 2D points.</B>
 * <TT>UUID: edcedc0a-9841-4377-aea1-9fa9fdb46fde</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolAR2DTransform : public org::bcom::xpcf::ComponentBase,
        public api::geom::I2DTransform {
public:

    SolAR2DTransform();
   ~SolAR2DTransform() override;

    /// @brief This method applies a 2D transform to a set of 2D points
    /// @param[in] inputPoints The 2D points on which the 2D transform will be applied.
    /// @param[in] transformation The 2D transform to apply to the set of 2D points.
    /// @param[out] outputPoints The resulting 2D points after application of the 2D transform.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode transform(const std::vector<datastructure::Point2Df> & inputPoints, const datastructure::Transform2Df & transformation, std::vector<datastructure::Point2Df> & outputPoints) override;
    
    void unloadComponent () override final;


 private:


};

}
}
}

#endif // SOLAR2DTRANSFORM_H
