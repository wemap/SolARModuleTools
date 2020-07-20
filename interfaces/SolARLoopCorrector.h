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

#ifndef SOLARLOOPCORRECTOR_H
#define SOLARLOOPCORRECTOR_H

#include "api/loop/ILoopCorrector.h"
// #include "api/reloc/IKeyframeRetriever.h"
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
 * @class SolARLoopCorrector
 * @brief TODO
 */
class SOLAR_TOOLS_EXPORT_API SolARLoopCorrector : public org::bcom::xpcf::ConfigurableBase,
        public api::loop::ILoopCorrector {
public:

    SolARLoopCorrector();
    ~SolARLoopCorrector() = default;

    /// @brief corrects a loop of keyframes and their associated point clouds from a loop detection result.
    /// @param[in] queryKeyframe: the query keyframe.
    /// @param[in] detectedLoopKeyframe: the detected loop keyframe.
    /// @param[in] S_c_wl : 3D similarity transformation (Sim(3)) from loop world c.s to reference keyframe c.s..
    // TODO adapt transformation on detector side ??? /// @param[out] sim3Transform : 3D similarity transformation (Sim(3)) from query keyframe to the detected loop keyframe.
    /// @param[in] duplicatedPointsIndices: indices of duplicated cloud points. The first index is the id of point cloud seen from the detected loop keyframe. The second one is id of point cloud seen from the query keyframe
    /// @return FrameworkReturnCode::_SUCCESS if loop closure is correctly corrected, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode correct(const SRef<Keyframe> &queryKeyframe, const SRef<Keyframe> &detectedLoopKeyframe, const Transform3Df &S_c_wl, const std::vector<std::pair<uint32_t, uint32_t>> &duplicatedPointsIndices) override;

    // virtual double correctsLoop(   const uint32_t reference_keyframe_id, const uint32_t loop_keyframe_id, const Transform3Df& S_c_wl) override;
     /*
     double SolARLoopCorrector::correctsLoop(   const uint32_t reference_keyframe_id, const uint32_t loop_keyframe_id, const Transform3Df& S_c_wl)
     {
         Vec<uint32_t> kfLoopNeighbors;
         Vec<uint32_t> kfCurrentNeighbors;

         // get covisible keyframes.
         getNeighbors(uint32_t kf_loop_id,    Vec<uint32_t> & kfLoopNeighboors );
         getNeighbors(uint32_t kf_current_id, Vec<uint32_t> & kfCurrentNeighboors);

         // Compute current keyframe's neighboors similarity poses in loop keyframe and current keyframe worlds c.s.
         // Let :
         // - T_c_wc be the pose of current keyframe in current keyframe world c.s.,
         // - T_wc_c be the inverse pose of current keyframe in current keyframe world c.s.,
         // - T_i_wc be the pose of the ith neighbors of current keyframe c.s.,
         // - T_i_c  = T_i_wc * T_wc_c be the SE3 transform from current keyframe c.s. to the ith neighbors of current keyframe c.s.
         // - S_i_c  = SE3_to_SIM3( T_i_c, 1) be the SIM3 transform from current keyframe c.s. to the ith neighbors of current keyframe c.s.
         // - S_c_wl be the SIM3 transform from loop keyframe world c.s. to the current keyframe c.s. (computed by loop detection component)
         // - S_i_wl = S_i_c * S_c_wl be the SIM3 pose i.e. the similarity pose of the ith neighbors of current keyframe (i.e. the transform from loop keyframe world c.s. to the ith neighbors of current keyframe c.s.)
         getNeighborhoodTransformedSimPoses(Vec<uint32_t> & kfCurrentNeighboors,  Map<uint32_t, Transform3Df > & KFSim_i_wls, Map<uint32_t, Transform3Df > & KFSim_i_wcs);  // CorrectedSim3/ Non Corrected

         // Transforms current keyframe neighboorhood observed points in loop keyframe world c.s.
         // both side of the loop connection will be expressed in loop keyframe world c.s.
         transformNeighborhoodPointsAndPoses(Map<uint32_t, Transform3Df > & KFSim_i_wls, Map<uint32_t, Transform3Df > & KFSim_i_wcs);

         // Merges points observed by both loop keyframe and current keyframe neighborhoods
         // update the covisibility graph according when a point merge occurs
         // compute the new loop connections map
         mergeNeighborhoodsPoints(Vec<uint32_t> kfLoopNeighboors, Vec<uint32_t> kfCurrentNeighboors, Map<uint32_t, set<uint32_t>> & mapLoopConnections);

         // Optimize essential graph
         optimizeEssentialGraph( const uint32_t reference_keyframe_id,
                                 const uint32_t loop_keyframe_id,
                                 const Transform3Df& S_c_wl,
                                 Map<uint32_t, Transform3Df > & KFSim_i_wcs,
                                 Map<uint32_t, Transform3Df > & KFSim_i_wls,
                                 Map<uint32_t, set<uint32_t>> & mapLoopConnections);


     }
     */
    
	void unloadComponent () override final;

 private:
    SRef<IKeyframesManager>								m_keyframesManager;
    SRef<ICovisibilityGraph>							m_covisibilityGraph;
    // SRef<reloc::IKeyframeRetriever>						m_keyframeRetriever;
    SRef<features::IDescriptorMatcher>					m_matcher;
    SRef<features::IMatchesFilter>						m_matchesFilter;
    // SRef<solver::pose::I3D3DCorrespondencesFinder>		m_corr3D3DFinder;
    SRef<geom::I3DTransform>							m_transform3D;
    //SRef<loop::ILoopOptimizer>                          m_loopOptimizer;



};

}
}
}

#endif // SOLARLOOPCORRECTOR_H
