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

#ifndef SOLAR3DTRANSFORMESTIMATIONFROM3D3D_H
#define SOLAR3DTRANSFORMESTIMATIONFROM3D3D_H
#include <vector>
#include <string>
#include "api/solver/pose/I3DTransformFinderFrom3D3D.h"
#include "datastructure/Image.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ComponentBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
* @class SolAR3DTransformEstimationFrom3D3D
* @brief <B>Finds the 3D transform of 3D-3D points correspondences.</B>
* <TT>UUID: 33f24be2-e2cc-4057-9c2b-a5bf865dc9f5</TT>
*
*/

class SOLAR_TOOLS_EXPORT_API SolAR3DTransformEstimationFrom3D3D : public org::bcom::xpcf::ComponentBase,
    public api::solver::pose::I3DTransformFinderFrom3D3D
{
public:
    ///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolAR3DTransformEstimationFrom3D3D();
    ///@brief SolAR3DTransformEstimationFrom3D3D destructor;
    ~SolAR3DTransformEstimationFrom3D3D();

	/// @brief Estimates 3D transformation including rotation, translation and scale from a set of 3D-3D point correspondences.
	/// @param[in] firstPoints3D: first set of 3D points.
	/// @param[in] secondPoints3D: second set of 3D points.
	/// @param[out] pose: 3D transformation maps the first set of 3D points to the second one.
	FrameworkReturnCode estimate(const std::vector<Point3Df> & firstPoints3D,
								const std::vector<Point3Df> & secondPoints3D,
								Transform3Df & pose) override;

    void unloadComponent () override final;
};

}
}
}

#endif // SOLAR3DTRANSFORMESTIMATIONFROM3D3D_H
