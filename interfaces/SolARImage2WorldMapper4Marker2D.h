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

#ifndef SOLARIMAGE2WORLDMAPPER4MARKER2D_H
#define SOLARIMAGE2WORLDMAPPER4MARKER2D_H

#include "api/geom/IImage2WorldMapper.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARImage2WorldMapper4Marker2D
 * @brief <B>Retrieves the 3D correspondences of pixels of a 2D marker.</B>
 * <TT>UUID: 6fed0169-4f01-4545-842a-3e2425bee248</TT>
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ digitalWidth,
 *                          the width of the 2D marker in pixels for a natural image marker\, or in cells for a squared binary marker,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ digitalHeight,
 *                          the height of the 2D marker in pixels for a natural image marker\, or in cells for a squared binary marker,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ worldWidth,
 *                          the width of the marker in a user-defined world coordinate system (meters\, cenimeters\, etc.),
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0 }}
 * @SolARComponentProperty{ worldHeight,
 *                          the height of the marker in a user-defined world coordinate system (meters\, cenimeters\, etc.),
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 0 }}
 * @SolARComponentPropertiesEnd
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARImage2WorldMapper4Marker2D : public org::bcom::xpcf::ConfigurableBase,
        public api::geom::IImage2WorldMapper {
public:

    SolARImage2WorldMapper4Marker2D();
    ~SolARImage2WorldMapper4Marker2D() override;
    /// @brief Retrieves the 3D correspondences of pixel of a 2D marker.
    /// @param[in] digitalPoints The 2D points defined in pixels in the 2D coordinate system of the marker .
    /// @param[in] worldPoints The 3D correspondences of the 2D points defined in the  world coordinate system.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode map(const std::vector<datastructure::Point2Df> & digitalPoints, std::vector<datastructure::Point3Df> & worldPoints) override;

    void unloadComponent () override final;


private:
    /// @brief the width of the 2D marker in pixels for a natural image marker, or in cells for a squared binary marker.
    int m_digitalWidth;

    /// @brief the height of the 2D marker in pixels for a natural image marker, or in cells for a squared binary marker.
    int m_digitalHeight;

    /// @brief the width of the marker in a user-defined world coordinate system (meters, cenimeters, etc.)
    float m_worldWidth;

    /// @brief the height of the marker in a user-defined world coordinate system (meters, cenimeters, etc.)
    float m_worldHeight;


};

}
}
}

#endif // SOLARIMAGE2WORLDMAPPER4MARKER2D_H
