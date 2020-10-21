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

#include "SolAROverlapDetector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAROverlapDetector);


namespace SolAR {
namespace MODULES {
namespace TOOLS {

SolAROverlapDetector::SolAROverlapDetector():ConfigurableBase(xpcf::toUUID<SolAROverlapDetector>())
{
    addInterface<api::loop::IOverlapDetector>(this);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
	declareInjectable<reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<solver::pose::I3DTransformSACFinderFrom3D3D>(m_estimator3D);
	declareInjectable<features::IDescriptorMatcher>(m_matcher, "Matcher-Loop");
	declareInjectable<features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<solver::pose::I3D3DCorrespondencesFinder>(m_corr3D3DFinder);
	declareInjectable<geom::I3DTransform>(m_transform3D);
	declareProperty("minNbInliers", m_NbMinInliers);
}

void SolAROverlapDetector::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_estimator3D->setCameraParameters(intrinsicParams, distortionParams);
}

FrameworkReturnCode SolAROverlapDetector::setGlobalMapper(const SRef<api::solver::map::IMapper>& map){
	map->getKeyframesManager(m_keyframesManager);
	map->getCovisibilityGraph(m_covisibilityGraph);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAROverlapDetector::detect(SRef<api::solver::map::IMapper> &globalMap, const SRef<api::solver::map::IMapper> &floatingMap, Transform3Df &sim3Transform) {

	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
