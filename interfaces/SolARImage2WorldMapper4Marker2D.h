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
#include "ComponentBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARImage2WorldMapper4Marker2D : public org::bcom::xpcf::ComponentBase,
        public api::geom::IImage2WorldMapper {
public:

    SolARImage2WorldMapper4Marker2D();
   ~SolARImage2WorldMapper4Marker2D();

    void setParameters(const Sizei digitalSize, const Sizef worldSize);
    FrameworkReturnCode map(const std::vector<SRef<Point2Df>> & digitalPoints, std::vector<SRef<Point3Df>> & worldPoints);

    void unloadComponent () override final;

    XPCF_DECLARE_UUID("6fed0169-4f01-4545-842a-3e2425bee248");

private:
   Sizei m_digitalSize;
   Sizef m_worldSize;

};

}
}
}

#endif // SOLARIMAGE2WORLDMAPPER4MARKER2D_H
