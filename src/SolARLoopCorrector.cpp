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

void SolARLoopCorrector::getNeighborhoodTransformedSimPoses(const uint32_t kfCurrentId,
                                                            const std::vector<uint32_t> & kfCurrentNeighbors,
                                                            const Transform3Df & S_c_wl,
                                                            std::map<uint32_t, Transform3Df > & KFSim_i_wls,
                                                            std::map<uint32_t, Transform3Df > & KFSim_i_wcs)
{
    // Compute current keyframe's neighboors similarity poses in loop keyframe and current keyframe worlds c.s.
    // Let :
    // - T_c_wc be the inverse SolAR pose of current keyframe in current keyframe world c.s.,
    // - T_wc_c be the SolAR pose of current keyframe in current keyframe world c.s.,
    // - T_i_wc be the SolAR pose of the ith neighbors of current keyframe c.s.,
    // - T_i_c  = T_i_wc * T_wc_c be the SE3 transform from current keyframe c.s. to the ith neighbors of current keyframe c.s.
    // - T_i_c  = T_i_wi * T_wi_wc * T_wc_c
    // - T_i_c  = T_i_wi * T_wc_c (since T_wi_wc is assumed to be Identity transform)
    // assuming that T_wi_wc = Id
    // - S_i_c  = SE3_to_SIM3( T_i_c, 1) be the SIM3 transform from current keyframe c.s. to the ith neighbors of current keyframe c.s.
    // - S_c_wl be the SIM3 transform from loop keyframe world c.s. to the current keyframe c.s. (computed by loop detection component)
    // - S_i_wl = S_i_c * S_c_wl be the SIM3 inverse pose i.e. the similarity pose of the ith neighbors of current keyframe (i.e. the transform from loop keyframe world c.s. to the ith neighbors of current keyframe c.s.)
    // S_wl_wc comes from detection

    // note : SolAR pose is equivalent to transform T_w_c (c->w).
    SRef<Keyframe> keyframe_current;
    SRef<Keyframe> keyframe_neighbor;
    Transform3Df   T_wc_c ; // current keyframe SolAR pose
    Transform3Df   T_wi_i ; // neighbor keyframe pose
    Transform3Df   T_i_wi ; // neighbor keyframe inverse SolAR pose

    //
    m_keyframesManager->getKeyframe(kfCurrentId, keyframe_current);
    T_wc_c = keyframe_current->getPose();

    //
    for(int i=0; i < kfCurrentNeighbors.size(); i++){
        uint32_t neighb_id = kfCurrentNeighbors[i];
        // get the neighbor pose
        m_keyframesManager->getKeyframe(neighb_id, keyframe_neighbor);
        T_wi_i = keyframe_neighbor->getPose();
        T_i_wi = T_wi_i.inverse();
        Transform3Df S_i_c, S_i_wl, S_i_wc;
        S_i_c  = T_i_wi * T_wc_c;
        S_i_wl = S_i_c  * S_c_wl;
        S_i_wc = T_i_wi; // Since : T_i_wc = T_i_wi * T_wi_wc = T_i_wi * Id = T_i_wi
        KFSim_i_wls.insert( std::pair<uint32_t, Transform3Df>(neighb_id, S_i_wl) );
        KFSim_i_wcs.insert( std::pair<uint32_t, Transform3Df>(neighb_id, S_i_wc) );
    }

    return;
}


FrameworkReturnCode SolARLoopCorrector::correct(const SRef<Keyframe> & queryKeyframe, const SRef<Keyframe> & detectedLoopKeyframe, const Transform3Df & S_c_wl, const std::vector<std::pair<uint32_t, uint32_t>> & duplicatedPointsIndices)
{
    // Get current and loop neighbors
    std::vector<uint32_t> kfLoopNeighbors;
    std::vector<uint32_t> kfCurrentNeighbors;
    uint32_t kfCurrentId = queryKeyframe->getId();
    uint32_t kfLoopId    = detectedLoopKeyframe->getId();
    m_covisibilityGraph->getNeighbors(kfCurrentId, 1.0, kfCurrentNeighbors);
    m_covisibilityGraph->getNeighbors(kfLoopId, 1.0, kfLoopNeighbors);

    // Compute current keyframe's neighboors similarity poses in loop keyframe and current keyframe worlds c.s.
    std::map<uint32_t, Transform3Df > KFSim_i_wls;
    std::map<uint32_t, Transform3Df > KFSim_i_wcs;
    getNeighborhoodTransformedSimPoses(kfCurrentId,
                                       kfCurrentNeighbors,
                                       S_c_wl,
                                       KFSim_i_wls,
                                       KFSim_i_wcs);

    //

    return FrameworkReturnCode::_SUCCESS;
}


}
}
}
