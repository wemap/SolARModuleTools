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

#ifndef SOLARLOOPCLOSUREDETECTOR_H
#define SOLARLOOPCLOSUREDETECTOR_H

#include "api/loop/ILoopClosureDetector.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"
#include "api/geom/I3DTransform.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3D3DCorrespondencesFinder.h"
#include "api/features/IMatchesFilter.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"
#include <fstream>
#include <mutex>

namespace SolAR {
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace MODULES {
namespace TOOLS {
/**
* @class SolARLoopClosureDetector
* @brief Detect a loop closure from a given keyframe.
* <TT>UUID: e3d5946c-c1f1-11ea-b3de-0242ac130004</TT>
*/
class SOLAR_TOOLS_EXPORT_API SolARLoopClosureDetector : public org::bcom::xpcf::ConfigurableBase,
        public api::loop::ILoopClosureDetector {
public:
	SolARLoopClosureDetector();
	~SolARLoopClosureDetector() = default;

	/// @brief Detect a loop closure from a given keyframe.
	/// @param[in] queryKeyframe: the query keyframe.
	/// @param[out] detectedLoopKeyframe: the detected loop keyframe.		
	/// @param[out] sim3Transform : 3D similarity transformation (Sim(3)) from query keyframe to the detected loop keyframe.
	/// @param[out] duplicatedPointsIndices: indices of duplicated cloud points. The first index is the id of point cloud seen from the detected loop keyframe. The second one is id of point cloud seen from the query keyframe
	/// @return FrameworkReturnCode::_SUCCESS if detect a loop closure, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode detect(const SRef<Keyframe> &queryKeyframe, SRef<Keyframe> &detectedLoopKeyframe, Transform3Df &sim3Transform, std::vector<std::pair<uint32_t, uint32_t>> &duplicatedPointsIndices) override;

	void unloadComponent () override final;

 private:
	SRef<IKeyframesManager>								m_keyframesManager;
	SRef<ICovisibilityGraph>							m_covisibilityGraph;
	SRef<reloc::IKeyframeRetriever>						m_keyframeRetriever;
	SRef<solver::pose::I3DTransformSACFinderFrom3D3D>	m_estimator3D;
	SRef<features::IDescriptorMatcher>					m_matcher;
	SRef<features::IMatchesFilter>						m_matchesFilter;
	SRef<solver::pose::I3D3DCorrespondencesFinder>		m_corr3D3DFinder;
	SRef<geom::I3DTransform>							m_transform3D;
	int													m_NbMinInliers;
};

}
}
}

#endif // SOLARLOOPCLOSUREDETECTOR_H
