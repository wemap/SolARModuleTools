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


#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"
#include <fstream>

#include <mutex>


namespace SolAR {
// using namespace datastructure;
namespace MODULES {
namespace TOOLS {


/**
 * @class SolARLoopCorrector
 * @brief TODO
 */
class SOLAR_TOOLS_EXPORT_API SolARLoopCorrector : public org::bcom::xpcf::ComponentBase,
        public api::loop::ILoopCorrector {
public:

    SolARLoopCorrector();
    ~SolARLoopCorrector() = default;

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
   

};

}
}
}

#endif // SOLARLOOPCORRECTOR_H
