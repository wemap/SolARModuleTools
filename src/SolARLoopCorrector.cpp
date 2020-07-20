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

#include "SolARLoopCorrector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARLoopCorrector);


namespace SolAR {
//using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARLoopCorrector::SolARLoopCorrector():ConfigurableBase(xpcf::toUUID<SolARLoopCorrector>())
{
    addInterface<api::loop::ILoopCorrector>(this);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
    declareInjectable<features::IDescriptorMatcher>(m_matcher);
    declareInjectable<features::IMatchesFilter>(m_matchesFilter);
    declareInjectable<geom::I3DTransform>(m_transform3D);
}

FrameworkReturnCode SolARLoopCorrector::correct(const SRef<Keyframe> & queryKeyframe, const SRef<Keyframe> & detectedLoopKeyframe, const Transform3Df & S_c_wl, const std::vector<std::pair<uint32_t, uint32_t>> & duplicatedPointsIndices)
{
    return FrameworkReturnCode::_SUCCESS;
}


}
}
}
